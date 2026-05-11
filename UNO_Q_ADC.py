from arduino.app_utils import *
import time
last_print=0
interval=.5
latest_adc=0;
def on_adc():#callback
  global latest_adc
  latest_adc=value
Bridge.provide("adc data", on_adc)

def loop():
  global last_print, latest_adc
  current_time = time.time()
  #Check every 500ms if there is new data
  if latest_adc is not None and current_time-last_print >= interval:
    voltage = latest_adc * (3.3 / 1023.0)
    print("ADC: {latest_adc}, Voltage: {voltage:.3f} V")
    last_print = current_time
App.run(user_loop=loop)
