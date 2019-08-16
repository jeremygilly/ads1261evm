# ads1261evm
How to use a Texas Instruments ADS1261 Evaluation Module

To use this custom module, first download it from GitHub:
```
git clone git://github.com/jeremygilly/ads1261evm.git
```
Before using the module, copy where you have saved it:
```
pwd
```
Then use the following to import the module:
```
import sys
sys.path.append('/path/that/you/copied/ads1261evm')
import ads1261evm
import time
import numpy as np
import spidev
import sys
import RPi.GPIO as GPIO
import matplotlib.pyplot as plt
from operator import itemgetter, attrgetter
import csv
import pandas as pd
import ntpath
import itertools
```

Success! Now you have access to the functions. Some basic functions to take measurements in Python 3:
```
adc = ads1261evm.ADC1261()
time.sleep(0.1) # allows the internal reference to settle.
DeviceID, RevisionID = adc.check_ID() # Prints the device that you have.
adc.choose_inputs(positive = 'AIN3', negative = 'AIN4') # choose the name of the inputs.
while (True):
	response=adc.collect_measurement()
	print("Response:", response,"mV")
	time.sleep(0.5)
```

Other function descriptions coming soon. For more information, please read ads1261evm.py for other possible functions.

