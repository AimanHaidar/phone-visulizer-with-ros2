# Phone Visualizer with ROS2
these packages give you the ability to visualze your phone with rviz2 and ros2 nodes and give you access to your phone rotation vector sensor instead of buying gryosope and make new application to extract rotation vector from angular speed

## requierments
- ros2
- phonesensor python package
- SensorStreamer Android App https://apkpure.com/sensorstreamer/cz.honzamrazek.sensorstreamer/downloadApplication
## how to run
first you should set the SensorStreamer app:

- add new connection from mange connection in the menu and set port greater than 1024, because less is used by android

- add new packet from manage packets

then you should get your phone IP from setting->About phone->IP Adress.

after that you set the app, you should configure the ros2 packages with yaml file.
go to src->vis_phone->config->phone_orient_params.yaml then edit the IP adress and the port
```yaml
/phonesensor_node:
  ros__parameters:
    IP_address: "192.168.0.75"
    port: 8080
```

then install the phonesensors python package with 

```bash
pip install phonesensors
```
or this if you are using uv and venv
```bash
    uv pip install -r requirements.txt
```
after installing every thing go to the repo path from terminal and build and source the packages with 
```bash
colcon build
source ./install/setup.bash
```
now you are ready to start the program just run 
```bash
ros2 launch vis_phone display.launch.py
```
and click start on the SensorStreamer app to start streaming sensors data

note: don't forget to source your ros2





