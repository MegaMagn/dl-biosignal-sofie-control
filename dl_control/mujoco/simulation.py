from __future__ import annotations

# Manage datasets
import numpy as np

# Manage file paths
from pathlib import Path
import os
import json

# plot
import matplotlib.pyplot as plt
from collections import deque, Counter

# Syncronization 
from time import time, perf_counter, sleep

# Mujoco
from myosuite.utils import gym
import msvcrt

# Own implementations
from src.experiment.real_time_operation import EMGRealTime, Buffer, Model, PredictionVoting, State, SimulationStateLogic, MujocoPredictionVoting, load_exisiting_datasets
from src.models.classification_pipeline import EMGStreamProcessor

#==================#
# Global variables #
#==================#
EMG_FREQ = 2000
EEG_FREQ = 125
RMS_FREQ = 40                   # 40 for 500 samples, 125 for 32 samples (window)

EEG_USEABLE_CHANNELS = [2, 3, 6, 7, 8, 9, 10, 11]

EMG_LOWCUT = 20
EMG_HIGHCUT = 450
EEG_LOWCUT = 0.5
EEG_HIGHCUT = 30

EEG_NUM_CH = len(EEG_USEABLE_CHANNELS)
EMG_NUM_CH = 3

RMS_SAMPLING_WINDOW = 500           # 500 samples - 250 ms                      32 samples - 16 ms                                       
RMS_WINDOW_STEPSIZE = 50            # 50 samples - 25 ms (90 % overlap)         16 samples - 8 ms (50 % overlap)

HAMPEL_WINDOWSIZE = 100
HAMPEL_SIGMA = 2                    # Usually 2

SLIDING_WINDOW_SAMPLES = 1000
SLIDING_WINDOW_STEPSIZE = 200

EMG_SELECT_SENSORS = (0, 2)
EMG_SAMPLES_PER_READ = 200

state = "REST"

EMG_CONFIG_DICT = {
    'rms_windowsize' : RMS_SAMPLING_WINDOW,
    'rms_stepsize' : RMS_WINDOW_STEPSIZE,
    'hampel_windowsize' : HAMPEL_WINDOWSIZE,
    'hampel_sigma' : HAMPEL_SIGMA,
    'hampel_plot_option' : [False, None],
    'include_EMG' : False
}

REJECT_CONFIG_DICT = {
    'EEG_epoch_rejection_tolerance' : 6,
    'EMG_epoch_rejection_tolerance' : 6,
    'EEG_ch_acceptance' : 0,
    'EMG_ch_acceptance' : 0
}

def prepare_joints(mujoco_model, env_data, env, actions, THUMB_JOINT):
    #======================#
    # Constrain all joints #
    #======================#

    original_joint_ranges = mujoco_model.jnt_range.copy()        # save original joint limits

    for i in range(mujoco_model.njnt):
            q = mujoco_model.jnt_qposadr[i]
            current = env_data.qpos[q]

            mujoco_model.jnt_range[i] = [current, current]       # Lock joints in place 


    #================================#
    # Move thumb to desired position #
    #================================#
    _unfreeze_joints(model = mujoco_model, joints_list = THUMB_JOINT, OJR = original_joint_ranges)     
    actions[22], actions[23] = 0.3, 0.1         # Move thumb

    t0 = time()
    try: 
        while env_data.qpos[4] > -0.6:
            env.mj_render()                       # Render the current simulation frame
                
            env.step(actions)    
            
            if time() - t0 > 5:
                raise TimeoutError('thumb location not found')
    finally:
        actions[22], actions[23] = 0, 0

        for i in range(mujoco_model.njnt):
            q = mujoco_model.jnt_qposadr[i]
            current = env_data.qpos[q]

            mujoco_model.jnt_range[i] = [current, current]
        
        freezed_joint_ranges = mujoco_model.jnt_range.copy()        # save freezed joints ranges
    
    return freezed_joint_ranges, original_joint_ranges

def start_simulation(model_folder_name):

    env = gym.make("myoHandPoseFixed-v0")
    env.reset()

    mujoco_model = env.sim.model
    env_data = env.sim.data
 
    NoC = mujoco_model.nu                         # Number of controls
    actions = np.zeros(NoC)                # Control vector with actuators

    #=====================#
    # Contract all joints #
    #=====================#

    freezed_joint_ranges, original_joint_ranges = prepare_joints(mujoco_model = mujoco_model,
                                                                env_data = env_data,
                                                                env = env,
                                                                actions = actions,
                                                                THUMB_JOINT = [3,4,5,6])
    

    all_joints = [joint for joint in range(23)]

    #=====================#
    # Prediction pipeline #
    #=====================#
    model_path_folder = Path(__file__).resolve().parents[1] / f"src/models/loggings/real_time/{model_folder_name}"

    MODEL = Model(path_to_model = model_path_folder, num_motions = 7)

    STREAM = EMGRealTime(config_dict = EMG_CONFIG_DICT,
                      select_sensors = EMG_SELECT_SENSORS,
                      samples_per_read = EMG_SAMPLES_PER_READ)
    
    EMG_BUFFER = Buffer(max_size = 10000,
                        num_ch = EMG_NUM_CH,
                        window_size = SLIDING_WINDOW_SAMPLES,
                        step_size = SLIDING_WINDOW_STEPSIZE)
    
    PREPROCESS = EMGStreamProcessor(fs = EMG_FREQ, lowcut = EMG_LOWCUT, highcut = EMG_HIGHCUT,
                                    reject_config_dict = EMG_CONFIG_DICT, 
                                    rms_window = RMS_SAMPLING_WINDOW, rms_step = RMS_WINDOW_STEPSIZE,
                                    hampel_window = HAMPEL_WINDOWSIZE, hampel_sigma = HAMPEL_SIGMA,     # sigma usually 2
                                    base_dir = 'Unused')
    
    # VOTER = PredictionVoting(window_size = 5, required_votes = 3)
    VOTER = MujocoPredictionVoting(window_size = 5, required_votes = 3)

    STATE = SimulationStateLogic()
    STREAM.start_stream()                      # Initilize streaming
    
    buffer_fill_size = 1
    mu = np.load(model_path_folder / "mu.npy")
    sigma = np.load(model_path_folder / "sigma.npy")

    # Initilize condictions for the first iteration when no prediction is made
    state = STATE.state

    pose_filename = Path(__file__).resolve().parents[1] / "src/utilities/motion_pose.json"
    with open(pose_filename, "r") as f:
        poses = json.load(f)

    start_time = perf_counter()
    last_update = perf_counter()
    spinner = ['|', '/', '-', '\\']
    spin_idx = 0

    total = 0

    # Log EMG data
    EMG_log_path = Path(__file__).resolve().parent / "data/flex_indexDemo_finger.csv"
    file_handle = open(EMG_log_path, 'a', buffering=1)

    try:
        while True:
            #====================================#
            # Load data -> preprocess -> predict # 
            #====================================#
            t0 = perf_counter()
            X_emg = STREAM.extract_data()               # Read data                        
            EMG_BUFFER.add_data(data = X_emg)           # Load into circular buffer

            np.savetxt(file_handle, X_emg, delimiter=',', fmt='%.6f')
            
            if buffer_fill_size < 5:
                buffer_fill_size += 1
                continue
            
            X_win = EMG_BUFFER.get_window()             # Extract window of data by sliding window
            X_pre = PREPROCESS.update(chunk = X_win)    # Preprocess window of data

            if X_pre is None:
                # print('Pre is none')
                continue
            
            X_norm = (X_pre - mu) / (sigma + 1e-8)                       # Normalize
            X_pred, confidence = MODEL.predict(input_data = X_norm)      # Insert into model

            X_avg_potential = np.mean(X_norm)

            total += 1

            finished = False

            final_pred = VOTER.update(X_pred, confidence = confidence, mujoco_state_obj = state)

            #==============#
            # Motion Check #
            #==============#
            if state.mode != 'REST':        
                if state.mode == 'ACTIVE':
                    target_pose = poses[state.motion].values()
                elif state.mode == 'RETURN':
                    target_pose = freezed_joint_ranges[:, 0]

                # Check if motion has reached final position, then return True
                finished = pose_reached(env_data = env_data, target_pose = target_pose, tolerance = 0.03)

            #==============#
            # Update state #
            #==============#
            STATE.update(pred = final_pred, avg_potential = X_avg_potential, finished = finished)        # State updates based on prediction and current situation
            state = STATE.state

            #=========#
            # Actuate #
            #=========#
            actions[:] = 0

            # 
            if state.init_pose or state.mode == 'RETURN':
                _freeze_joints(model = mujoco_model, joints_list = all_joints, original_joints = freezed_joint_ranges)
                env.mj_render()                       # Render the current simulation frame
                env.step(actions)                     # performs a physics step

            # move joints when there is action
            elif state.mode != 'REST' and not finished:
                
                motion = state.motion_dict[state.motion]

                _unfreeze_joints(model = mujoco_model, joints_list = motion['joints'], OJR = original_joint_ranges)

                if state.mode == 'ACTIVE':
                    target_pose = poses[state.motion]
                    apply_motion_pose(model = mujoco_model, target_pose = target_pose)                
            
                env.mj_render()                       # Render the current simulation frame
                env.step(actions)                     # performs a physics step
            
                
            #=====================#
            # Timing / Monitoring #
            #=====================#
            current = perf_counter()

            elapsed = current - start_time

            # dt = current - last_update

            # last_update = current

            loop_time_ms = (current - t0) * 1000

            fps = total / elapsed

            # Detect loop stalls
            # if dt > 0.5:
            #     print(f"\nWARNING: Loop stalled for {dt:.2f}s")

            # Spinner animation
            symbol = spinner[spin_idx % len(spinner)]
            spin_idx += 1

            #================#
            # Live printout  #
            #================#
            print(
                f"{symbol} "
                f"TIME: {elapsed:7.1f}s | "
                f"ITER: {total:<6} | "
                f"STATE: {state.mode:<10} | "
                f"PRED: {X_pred:<20} | "
                # f"RAW: {VOTER.get_buffer()} | "
                f"FINAL: {final_pred} | "
                f"CONF: {confidence:>5.2f} | "
                f"LOOP: {loop_time_ms:>6.1f} ms | "
                f"FPS: {fps:>5.1f}",
                end="\r",
                flush=True
            )
            # sleep(0.01)

    except KeyboardInterrupt:
        print("\nSimulation stopped by user.")

    finally:
        env_data.close()
        STREAM.end_stream()
        # plt.ioff()

def apply_motion_pose(model, target_pose):

    for joint_idx, value in target_pose.items():
        j = int(joint_idx)

        model.jnt_range[j] = [value, value]

def pose_reached(env_data, target_pose, tolerance=0.03):

    for joint_idx, target_value in enumerate(target_pose):
    #for joint_idx, target_value in target_pose.items():

        current = env_data.qpos[int(joint_idx)]

        error = abs(current - target_value)

        if error > tolerance:
            return False

    return True

def _freeze_joints(model, joints_list, original_joints):
    # print('Original_joints')
    # print(original_joints)
    for j in joints_list:
        model.jnt_range[j] = original_joints[j]

def _unfreeze_joints(model, joints_list, OJR):
    for j in joints_list:
        model.jnt_range[j] = OJR[j]


def demo_inspect_latency(model_folder_name, num_motions : int = 7, demo_motion : str = 'none'):

    env = gym.make("myoHandPoseFixed-v0")
    env.reset()

    mujoco_model = env.sim.model
    env_data = env.sim.data
 
    NoC = mujoco_model.nu                         # Number of controls
    actions = np.zeros(NoC)                # Control vector with actuators

    #=====================#
    # Contract all joints #
    #=====================#

    freezed_joint_ranges, original_joint_ranges = prepare_joints(mujoco_model = mujoco_model,
                                                                env_data = env_data,
                                                                env = env,
                                                                actions = actions,
                                                                THUMB_JOINT = [3,4,5,6])
    

    all_joints = [joint for joint in range(23)]

    #=====================#
    # Prediction pipeline #
    #=====================#
    model_path_folder = Path(__file__).resolve().parents[1] / f"src/models/loggings/real_time/{model_folder_name}"

    MODEL = Model(path_to_model = model_path_folder, num_motions = num_motions)

    # STREAM = EMGRealTime(config_dict = EMG_CONFIG_DICT,
    #                   select_sensors = EMG_SELECT_SENSORS,
    #                   samples_per_read = EMG_SAMPLES_PER_READ)
    
    EMG_BUFFER = Buffer(max_size = 10000,
                        num_ch = EMG_NUM_CH,
                        window_size = SLIDING_WINDOW_SAMPLES,
                        step_size = SLIDING_WINDOW_STEPSIZE)
    
    PREPROCESS = EMGStreamProcessor(fs = EMG_FREQ, lowcut = EMG_LOWCUT, highcut = EMG_HIGHCUT,
                                    reject_config_dict = EMG_CONFIG_DICT, 
                                    rms_window = RMS_SAMPLING_WINDOW, rms_step = RMS_WINDOW_STEPSIZE,
                                    hampel_window = HAMPEL_WINDOWSIZE, hampel_sigma = HAMPEL_SIGMA,     # sigma usually 2
                                    base_dir = 'Unused')
    
    # VOTER = PredictionVoting(window_size = 5, required_votes = 3)
    VOTER = MujocoPredictionVoting(window_size = 5, required_votes = 3)

    STATE = SimulationStateLogic()

    X_epoch = load_exisiting_datasets(num_motions = num_motions, demo_motion = demo_motion)
    
    buffer_fill_size = 1
    mu = np.load(model_path_folder / "mu.npy")
    sigma = np.load(model_path_folder / "sigma.npy")

    # Initilize condictions for the first iteration when no prediction is made
    state = STATE.state

    pose_filename = Path(__file__).resolve().parents[1] / "src/utilities/motion_pose.json"
    with open(pose_filename, "r") as f:
        poses = json.load(f)


    total = 0

    #=======================#
    # Handle data structure #
    #=======================#
    buffer_fill_size = 1
    WINDOW = 1000
    STEP = 200

    if num_motions == 3:
        motion_list = ['Index', 'Thumb', 'Pinch']
    elif num_motions == 7:
        motion_list = ['Cylinder', 'Middle', 'Middle', 'Index', 'Thumb', 'Pinch', 'Cylinder']
    
    label_order = []
    for limb in motion_list:
        for act in ['Contract', 'Release']:
            comb = limb + ' ' + act
            label_order.append(comb)
    label_order.append('Rest')

    demo_ranges = {
        'Index': slice(160*EMG_FREQ, None),
        'Middle': slice(53*EMG_FREQ, 105*EMG_FREQ),
        'Ring': slice(0*EMG_FREQ, None),               # 18
        'Pinky': slice(93*EMG_FREQ, None),
        'Pinch': slice(272*EMG_FREQ, None),
        'Cylinder': slice(0, None),
        'Thumb': slice(30*EMG_FREQ, None),
    }
    demo_latency_dict = {}

    input('Wait for Enter')
    onset_flag = 'return_rest'
    onset_previous_motion = None
    motion_iter = 0
    motion_iter_flag = [False, False]
    ignore_first_release = False

    logger_template = {
    'thumb' : np.zeros(20),
    'index' : np.zeros(20),
    'middle' : np.zeros(20),
    'ring' : np.zeros(20),
    'pinky' : np.zeros(20),
    'pinch' : np.zeros(20),
    'cylinder' : np.zeros(20),
    }


    import copy

    on_latency_logger = copy.deepcopy(logger_template)
    on_correction_logger = copy.deepcopy(logger_template)
    off_latency_logger = copy.deepcopy(logger_template)
    off_correction_logger = copy.deepcopy(logger_template)

    try:
        for ml in motion_list:
            if demo_motion != 'none' and num_motions == 7:
                motion_data = X_epoch['subject_0'][ml]
                motion_data = motion_data[demo_ranges[ml], :]
                demo_latency_dict[ml] = {}
                demo_latency_dict[ml]['ACTIVE'] = []
                demo_latency_dict[ml]['RETURN'] = []
                demo_single_execution = False
                t0 = None

                if ml == 'Middle':
                    print('Break when Ring is reached')
                    break
            #====================================#
            # Load data -> preprocess -> predict # 
            #====================================#
            for start in range(0, motion_data.shape[0] - WINDOW + 1, STEP):

                if t0 is None:
                    t_total = time()
                    pass
                else:
                    while time() - t0 < 0.1:
                        pass

                X_emg = motion_data[start : start + STEP]

                t0 = time()
          
                EMG_BUFFER.add_data(data = X_emg)           # Load into circular buffer

            
                if buffer_fill_size < 5:
                    buffer_fill_size += 1
                    continue
            
                X_win = EMG_BUFFER.get_window()             # Extract window of data by sliding window
                X_pre = PREPROCESS.update(chunk = X_win)    # Preprocess window of data

                if X_pre is None:
                    # print('Pre is none')
                    continue
                
                X_norm = (X_pre - mu) / (sigma + 1e-8)                       # Normalize
                X_pred, confidence = MODEL.predict(input_data = X_norm)      # Insert into model

                X_avg_potential = np.mean(X_norm)

                total += 1

                finished = False

                final_pred, pred_action = VOTER.update(X_pred, confidence = confidence)

                #===============#
                # Track latency #
                #===============#
                if X_pred is not None:
                    motion_name, action = extract_motion_action(prediction = X_pred)

                    if motion_name == 'none':
                        pass

                    else:
                        #
                        # During contraction
                        #
                        if action == 'contract':
                            # Set timer when first contraction
                            if onset_flag == 'return_rest' or onset_flag == 'return_rest_corrected':
                                t_onset = time()
                                onset_flag = 'onset'
                                onset_previous_motion = motion_name
                                ignore_first_release = True

                                if motion_iter_flag[0] and motion_iter_flag[1]:
                                    motion_iter_flag[0], motion_iter_flag[1] = False, False
                                    motion_iter += 1

                                    if motion_iter == 20:
                                        print('break outer loop')
                                        break

                            # Check for self-correction and log latency
                            if pred_action == 'contract':
                                # Look what the majority voting predicts
                                final_motion_name, final_action = extract_motion_action(prediction = final_pred)

                                if final_action != 'contract':
                                    raise ValueError(f'final_action is of {final_action} - Should be contract?')

                                # Log latency when contraction happens
                                if onset_flag == 'onset':
                                    t_diff = time() - t_onset
                                    on_latency_logger[motion_name][motion_iter] = t_diff
                                    motion_iter_flag[0] = True
                                    print(on_latency_logger)
                                    onset_flag = 'return_offset'

                                # Senario for self-correction
                                elif onset_flag == 'return_offset' and onset_previous_motion != final_motion_name:
                                    t_correct = (time() - t_onset) - t_diff
                                    on_correction_logger[final_motion_name][motion_iter] = t_correct
                                    print(on_correction_logger)
                                    onset_flag = 'return_offset_corrected'
                            
                        if action == 'release':
                            # Set timer when first release
                            if onset_flag == 'return_offset' or onset_flag == 'return_offset_corrected':
                                t_offset = time()
                                onset_flag = 'offset'
                                onset_previous_motion = motion_name
                            
                            if pred_action == 'release':
                                # Look what the majority voting predicts
                                final_motion_name, final_action = extract_motion_action(prediction = final_pred)

                                if final_action != 'release':
                                    raise ValueError(f'final_action is of {final_action} - Should be release?')
                                                                # Log latency when contraction happens
                                if onset_flag == 'offset':
                                    t_diff = time() - t_offset
                                    off_latency_logger[motion_name][motion_iter] = t_diff
                                    motion_iter_flag[1] = True
                                    print(off_latency_logger)
                                    onset_flag = 'return_rest'

                                elif onset_flag == 'return_rest' and onset_previous_motion != final_motion_name and ignore_first_release:
                                    t_correct = (time() - t_offset) - t_diff
                                    off_correction_logger[final_motion_name][motion_iter] = t_correct
                                    print(off_correction_logger)
                                    onset_flag = 'return_rest_corrected'

                #==============#
                # Motion Check #
                #==============#
                if state.mode != 'REST':        
                    if state.mode == 'ACTIVE':
                        target_pose = poses[state.motion].values()
                    elif state.mode == 'RETURN':
                        target_pose = freezed_joint_ranges[:, 0]

                    # Check if motion has reached final position, then return True
                    finished = pose_reached(env_data = env_data, target_pose = target_pose, tolerance = 0.03)

                #==============#
                # Update state #
                #==============#
                STATE.update(pred = final_pred, avg_potential = X_avg_potential, finished = finished)        # State updates based on prediction and current situation
                state = STATE.state

                #=========#
                # Actuate #
                #=========#
                actions[:] = 0

                # 
                if state.init_pose or state.mode == 'RETURN':
                    _freeze_joints(model = mujoco_model, joints_list = all_joints, original_joints = freezed_joint_ranges)
                    env.mj_render()                       # Render the current simulation frame
                    env.step(actions)                     # performs a physics step

                # move joints when there is action
                elif state.mode != 'REST' and not finished:
                    
                    motion = state.motion_dict[state.motion]

                    _unfreeze_joints(model = mujoco_model, joints_list = motion['joints'], OJR = original_joint_ranges)

                    if state.mode == 'ACTIVE':
                        target_pose = poses[state.motion]
                        apply_motion_pose(model = mujoco_model, target_pose = target_pose)                
                
                    env.mj_render()                       # Render the current simulation frame
                    env.step(actions)                     # performs a physics step

                #================#
                # Live printout  #
                #================#
                print(
                    f"ITER: {total:<6} | "
                    f"STATE: {state.mode:<10} | "
                    # f"ACT: {X_avg_potential:>5.2f} "|
                    f"PRED: {X_pred:<20} | "
                    # f"RAW: {VOTER.get_buffer()} | "
                    f"FINAL: {final_pred} | "
                    f"CONF: {confidence:>5.2f} | "
                    # end="\r",
                    # flush=True
                )
                # sleep(0.01)

    except KeyboardInterrupt:
        print("\nSimulation stopped by user.")

    finally:
        print(on_latency_logger, "\n\n")
        print(on_correction_logger, "\n\n")
        print(off_latency_logger, "\n\n")
        print(off_correction_logger, "\n\n")
        print(f"Total time: {time() - t_total:.2f} s")
        import pickle
        all_loggers = {
        'on_latency_logger': on_latency_logger,
        'on_correction_logger': on_correction_logger,
        'off_latency_logger': off_latency_logger,
        'off_correction_logger': off_correction_logger,
        }

        with open("latency_loggers.pkl", "wb") as f:
            pickle.dump(all_loggers, f)

        env_data.close()
        # STREAM.end_stream()
        # plt.ioff()

def extract_motion_action(prediction):
    parts = prediction.split()          # e.g. "Index Contract"
    if len(parts) == 2: 
        motion_name, action = prediction.split()
        motion_name = motion_name.lower()
        action = action.lower()
        return motion_name, action
    
    return 'none', 'none'
if __name__ == '__main__':
    
    model_folder_name = 'fine_tune/SingleNet_CNN+LSTM_EMG_newCrop_7motions/SingleNet_CNN+LSTM_EMG/subject_0'
    # start_simulation(model_folder_name = model_folder_name)
    demo_inspect_latency(model_folder_name = model_folder_name, num_motions = 7, demo_motion = 'all')

# Actuators
# idx name _
# 0 ECRL 0.0
# 1 ECRB 0.0
# 2 ECU 0.0
# 3 FCR 0.0
# 4 FCU 0.0
# 5 PL 0.0
# 6 PT 0.0
# 7 PQ 0.0
# 8 FDS5 0.0
# 9 FDS4 0.0
# 10 FDS3 0.0
# 11 FDS2 0.0
# 12 FDP5 0.0
# 13 FDP4 0.0
# 14 FDP3 0.0
# 15 FDP2 0.0
# 16 EDC5 0.0
# 17 EDC4 0.0
# 18 EDC3 0.0
# 19 EDC2 0.0
# 20 EDM 0.0
# 21 EIP 0.0
# 22 EPL 0.0            # extend thumb
# 23 EPB 0.0            # extend thumb
# 24 FPL 0.0            # bend thumb
# 25 APL 0.0            # extend thumb (maybe)
# 26 OP 0.0             # bend thumb (maybe)
# 27 RI2 0.0
# 28 LU_RB2 0.0
# 29 UI_UB2 0.0
# 30 RI3 0.0
# 31 LU_RB3 0.0
# 32 UI_UB3 0.0
# 33 RI4 0.0
# 34 LU_RB4 0.0
# 35 UI_UB4 0.0
# 36 RI5 0.0
# 37 LU_RB5 0.0
# 38 UI_UB5 0.0

# JOINTS
# idx name _
# 0 pro_sup 0.0
# 1 deviation 0.0
# 2 flexion 0.0
# 3 cmc_abduction 0.0
# 4 cmc_flexion 0.0
# 5 mp_flexion 0.0
# 6 ip_flexion 0.0
# 7 mcp2_flexion 0.0
# 8 mcp2_abduction 0.0
# 9 pm2_flexion 0.0
# 10 md2_flexion 0.0
# 11 mcp3_flexion 0.0
# 12 mcp3_abduction 0.0
# 13 pm3_flexion 0.0
# 14 md3_flexion 0.0
# 15 mcp4_flexion 0.0
# 16 mcp4_abduction 0.0
# 17 pm4_flexion 0.0
# 18 md4_flexion 0.0
# 19 mcp5_flexion 0.0
# 20 mcp5_abduction 0.0
# 21 pm5_flexion 0.0
# 22 md5_flexion 0.0