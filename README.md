# openFloat
This repository documents the build and implementation of a DIY profiling float for my undergraduate senior thesis at the University of Washington School of Oceanography.
To clone this repository to your working directory, paste the following line into your terminal or command line:
```
git clone https://github.com/cflaim1123/openFloat.git
```
If you do not have git installed on your machine, follow this [link](https://git-scm.com/downloads) to the git website. 

## Project description
This project aims to quantify haloclines in Colvos Passage Puget Sound, WA, by building a DIY profiling float that costs ~$500 to manufacture. Traditional autonomous underwater vehicles (AUVs) cost ~$100,000 or more. We hope to obtain high-quality data using traditional AUV deployment and propulsion methods with a DIY AUV. This reduces the financial barrier required to use AUV data collection methods for scientific research. 

The openFloat will collect 12-bit analog pressure and temperature, GPS location, lux, temperature, internal humidity, temperature, and pressure data. All data will be written to an onboard SD card and transmitted to an onshore base station via 915 mHz LoRa (long-range) radio. 

