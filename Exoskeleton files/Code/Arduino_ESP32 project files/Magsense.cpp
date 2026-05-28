#include "Magsense.h"
#include "config.h"
#include "tcp_server.h"
#include <Wire.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>


static inline void pcaSelect(uint8_t i) {
  if (i > 3) return;
  Wire.beginTransmission(PCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

SFE_MMC5983MA mag;

void initMagsense() {
  Wire.begin();
  Wire.setClock(400000);  // fast I2C
  pcaSelect(PCA_CHANNEL);

  if(!mag.begin(Wire)) {
    client.println("ERROR: MMC5983MA not detected");
  }
  mag.softReset();
  mag.enableAutomaticSetReset();
  mag.setFilterBandwidth(100);
  mag.setContinuousModeFrequency(100);
  mag.enableContinuousMode();
}

void readMagsense(double *vars){
  uint32_t x=0, y=0, z=0;
  double scaledX = 0, scaledY = 0, scaledZ = 0;
  pcaSelect(PCA_CHANNEL);
  bool ok = mag.getMeasurementXYZ(&x, &y, &z);
  if (!ok) return;

  scaledX = (double)x - 131072.0;
  scaledX /= 131072.0;
  scaledY = (double)y - 131072.0;
  scaledY /= 131072.0;
  scaledZ = (double)z - 131072.0;
  scaledZ /= 131072.0;
  vars[0] = scaledX;
  vars[1] = scaledY;
  vars[2] = scaledZ;
}
