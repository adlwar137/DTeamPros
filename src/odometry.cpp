#include "odometry.h"

void odometry_track(void* tracking_params) {
  while(1) {
    tracking_params_t* params = (tracking_params_t*)tracking_params;
    double leftDistance = ((double)(ext_adi_encoder_get(params->left_encoder)) / 360) * M_PI * TRACKING_WHEEL_DIAMETER;
    double rightDistance = ((double)(ext_adi_encoder_get(params->right_encoder)) / 360) * M_PI * TRACKING_WHEEL_DIAMETER;
    double strafeDistance = ((double)(ext_adi_encoder_get(params->strafe_encoder)) / 360) * M_PI * TRACKING_WHEEL_DIAMETER;

    double prevLeftDistance;
    double prevRightDistance;
    double prevStrafeDistance;
    double prevHeading;

    double deltaLeftDistance = leftDistance - prevLeftDistance;
    double deltaRightDistance = rightDistance - prevRightDistance;
    double deltaStrafeDistance = strafeDistance - prevStrafeDistance;

    //radians
    double deltaheading = (deltaLeftDistance - deltaRightDistance) / (LEFT_TRACKING_WHEEL_DISTANCE_FROM_CENTER + RIGHT_TRACKING_WHEEL_DISTANCE_FROM_CENTER);
    
    double heading = heading + deltaheading;

    double deltaXPosition;
    double deltaYPosition;

    double deltaGlobalXPosition;
    double deltaGlobalYPosition;

    deltaheading = 0;

    if(deltaheading <= 0.000001 && deltaheading >= -0.000001) {
      deltaXPosition = deltaStrafeDistance;
      deltaYPosition = deltaRightDistance;
    } else {
      deltaXPosition = 2*sin((heading/2)*(deltaStrafeDistance/deltaheading) + STRAFE_TRACKING_WHEEL_DISTANCE_FROM_CENTER);
      deltaYPosition = 2*sin((heading/2)*(deltaRightDistance/deltaheading) + RIGHT_TRACKING_WHEEL_DISTANCE_FROM_CENTER);
    }

    double averageHeading = prevHeading + (deltaheading/2);

    deltaGlobalXPosition = deltaXPosition*cos(-averageHeading) - deltaYPosition*sin(-averageHeading);
    deltaGlobalYPosition = deltaXPosition*sin(-averageHeading) + deltaYPosition*cos(-averageHeading);

    double GlobalXPosition = GlobalXPosition + deltaGlobalXPosition;
    double GlobalYPosition = GlobalYPosition + deltaGlobalYPosition;

    printf("X: %f, Y: %f, Heading: %f\n", GlobalXPosition, GlobalYPosition, heading);

    prevLeftDistance = leftDistance;
    prevRightDistance = rightDistance;
    prevStrafeDistance = strafeDistance;
    prevHeading = heading;

    params->pose.x = GlobalXPosition;
    params->pose.y = GlobalYPosition;
    params->pose.w = heading;
    
    //wait to free up resources
    delay(20);
  }
}