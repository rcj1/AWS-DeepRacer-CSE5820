# AWS DeepRacer CSE5820
Code for my best submission to the CSE 5820 AWS DeepRacer virtual race. For this model, I used the camera and LIDAR sensors and a learning rate of 0.0003. Model finished in 2nd place out of 26, within 0.7s of the top finisher.

### Race details
Race was held on the Po-Chun Speedway; below is an image of the track shape.

![image](https://github.com/rcj1/AWS-DeepRacer-CSE5820/assets/77995559/c2ffcf8c-cb51-4245-b57a-93e391d0a299)

### Code explanation
Much of this code is based off [this GitHub repo](https://github.com/falktan/deepracer). The basic concept is always pointing the car towards the waypoint that is several waypoints ahead on the track. I added code to reward the car for speed on straightaways; this helped improve the finishing time. For a more detailed explanation, head over to the above GitHub link.

