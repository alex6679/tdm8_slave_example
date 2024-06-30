#include "input_tdm2_slave.h"
#include "plotter.h"
#include <Audio.h>

AudioInputTDM2Slave            tdm;
Plotter                  plotter(4);  //only plot every 4th sample           
AudioConnection          patchCord1(tdm, 0, plotter, 0);
AudioConnection          patchCord2(tdm, 1, plotter, 1);

void setup() {
  AudioMemory(50);
  Serial.begin(115200);
  while(!Serial){};
  plotter.activate(true);
}

void loop() {
}