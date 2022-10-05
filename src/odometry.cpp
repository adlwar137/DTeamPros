#include "odometry.h"

double odometry_get_encoder_distance(adi_encoder_t encoder, double wheel_diameter) {
  return ((double)(ext_adi_encoder_get(encoder)) / 360) * M_PI * wheel_diameter;
}

void odometry_track(void* tracking_params) {
  tracking_params_t *paramsP = (tracking_params_t*)tracking_params;
  tracking_params_t params = *paramsP;
  while(1) {
    /*
    double leftDistance = ((double)(ext_adi_encoder_get(params->left_encoder)) / 360) * M_PI * TRACKING_WHEEL_DIAMETER;
    double rightDistance = ((double)(ext_adi_encoder_get(params->right_encoder)) / 360) * M_PI * TRACKING_WHEEL_DIAMETER;
    double strafeDistance = ((double)(ext_adi_encoder_get(params->strafe_encoder)) / 360) * M_PI * TRACKING_WHEEL_DIAMETER;
    */
    double leftDistance = odometry_get_encoder_distance(params.left_encoder, TRACKING_WHEEL_DIAMETER);
    double rightDistance = odometry_get_encoder_distance(params.right_encoder, TRACKING_WHEEL_DIAMETER);
    double strafeDistance = odometry_get_encoder_distance(params.strafe_encoder, TRACKING_WHEEL_DIAMETER);

    double prevLeftDistance;
    double prevRightDistance;
    double prevStrafeDistance;
    double prevHeading;

    double deltaLeftDistance = leftDistance - prevLeftDistance;
    double deltaRightDistance = rightDistance - prevRightDistance;
    double deltaStrafeDistance = strafeDistance - prevStrafeDistance;

    //radians

    //not calculating delta w
    double deltaHeading = (deltaLeftDistance - deltaRightDistance) / (LEFT_TRACKING_WHEEL_DISTANCE_FROM_CENTER + RIGHT_TRACKING_WHEEL_DISTANCE_FROM_CENTER);
    double heading = heading + deltaHeading;

    double deltaXPosition, deltaYPosition;
    double deltaGlobalXPosition, deltaGlobalYPosition;

    if(deltaHeading <= 0.000001 && deltaHeading >= -0.000001) {
      deltaXPosition = deltaStrafeDistance;
      deltaYPosition = deltaRightDistance;
    } else {
      deltaXPosition = 2*sin(deltaHeading/2)*(((deltaStrafeDistance/deltaHeading) + STRAFE_TRACKING_WHEEL_DISTANCE_FROM_CENTER));
      deltaYPosition = 2*sin(deltaHeading/2)*(((deltaRightDistance/deltaHeading) + RIGHT_TRACKING_WHEEL_DISTANCE_FROM_CENTER));
    }

    double averageHeading = prevHeading + (deltaHeading/2);

    deltaGlobalXPosition = deltaXPosition*cos(-averageHeading) - deltaYPosition*sin(-averageHeading);
    deltaGlobalYPosition = deltaXPosition*sin(-averageHeading) + deltaYPosition*cos(-averageHeading);

    double GlobalXPosition = GlobalXPosition + deltaGlobalXPosition;
    double GlobalYPosition = GlobalYPosition + deltaGlobalYPosition;

    //printf("X: %f, Y: %f, deltaHeading: %f\n", GlobalXPosition, GlobalYPosition, deltaHeading);

    prevLeftDistance = leftDistance;
    prevRightDistance = rightDistance;
    prevStrafeDistance = strafeDistance;
    prevHeading = heading;

    params.pose->x = GlobalXPosition;
    params.pose->y = GlobalYPosition;
    params.pose->w = heading;

    //printf("posX: %f, posY: %f, posW: %f\n", params.pose->x, params.pose->y, params.pose->w);

    //wait to free up resources
    delay(20);
  }
}