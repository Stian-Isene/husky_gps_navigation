# Service for Husky

## gps_user_service

This service communicates with the web app to save data points from user.

It saves the data to packet folder in `src` folder. In a txt file where each line is a gps poins `lat,lan`.

Use following comand to start service:
```
rosrun gps_user_service gps_user_service_server.py
```

Can be tested by using test client when service are running:
```
rosrun gps_user_service gps_user_service_client.py
```

### Message type 

Uses SaveGPS.srv to defind request and response message.

```
# Request
sensor_msgs/NavSatFix[] gps_coordinates
---
# Response
bool success
```

## read_file
A ROS pack that contains `get_data.py` for extracting the data saved by `gps_user_service` and return a list of tuples with gps coordinates.

Build as a packet and run using:
```
rosrun read_file get_data.py
```

Or use the function in `get_data.py` to retrive data from file in `gps_user_service`
