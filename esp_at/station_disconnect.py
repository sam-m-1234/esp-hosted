# Copyright 2019 Espressif Systems (Shanghai) PTE LTD
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from at_commands import at_commands
import argparse

parser = argparse.ArgumentParser(description='station_disconnect.py script will disconnect ESPStation from AP ex. python3 station_disconnect.py')

wifi_mode = at_commands.wifi_get_mode()
print(wifi_mode)

if (wifi_mode == '1'):
    disconnect = at_commands.wifi_disconnect_ap()
    print(disconnect)
    print("Disconnected from AP")
elif (wifi_mode == 'failure'):
    print("failure in disconnect")
else :
    print("station mode is not selected, current mode is "+str(wifi_mode))
    print("0: null Mode, Wi-Fi RF will be disabled")
    print("1: station mode")
    print("2: softAP mode")
    print("3: softAP+station mode")
