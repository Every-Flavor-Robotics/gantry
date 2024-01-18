// #include <math.h>

// #ifndef trajectory_utils_h
// #define trajectory_utils_h

// #define MAX_TRAJECTORY_SIZE 3000

// // Need to define a datatype to hold waypoints
// // 2D only for now
// typedef struct
// {
//   float q0;
//   float q1;
// } waypoint_t;

// int get_trajectory_len(waypoint_t start, waypoint_t end, double resolution)
// {
//   // Calculate the angular distance between the waypoints
//   double q0_dist = end.q0 - start.q0;
//   double q1_dist = end.q1 - start.q1;

//   // Choose the larger of the two distances
//   double max_dist =
//       (fabs(q0_dist) > fabs(q1_dist)) ? fabs(q0_dist) : fabs(q1_dist);

//   // Calculate the length of the path
//   int path_length = max_dist / resolution;

//   // If path length is greater than the maximum allowed, scale the resolution
//   if (path_length > MAX_TRAJECTORY_SIZE)
//   {
//     resolution = max_dist / MAX_TRAJECTORY_SIZE;
//     path_length = MAX_TRAJECTORY_SIZE;
//   }

//   return path_length;
// }

// // linear_interpolate: linearly interpolate between waypoints to create a
// path
// // The waypoints will be defined such that the angular distance between them
// is
// // is constant. Additionally, both joints will reach the end of their path at
// // the same time.
// // start: starting waypoint
// // end: ending waypoint
// // path: array of waypoints to store the path
// // resolution: angular resolution of the path
// void linear_interpolate(waypoint_t start, waypoint_t end, waypoint_t *path,
//                         double resolution)
// {
//   // Calculate the angular distance between the waypoints
//   double q0_dist = end.q0 - start.q0;
//   double q1_dist = end.q1 - start.q1;

//   // Choose the larger of the two distances
//   double max_dist =
//       (fabs(q0_dist) > fabs(q1_dist)) ? fabs(q0_dist) : fabs(q1_dist);

//   // Calculate the length of the path
//   int path_length = get_trajectory_len(start, end, resolution);

//   // Compute step size for each joint
//   double q0_step = q0_dist / path_length;
//   double q1_step = q1_dist / path_length;

//   // Create array of waypoints
//   for (int i = 0; i < path_length; i++)
//   {
//     path[i].q0 = start.q0 + q0_step * i;
//     path[i].q1 = start.q1 + q1_step * i;
//   }
// }

// #endif