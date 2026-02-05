#ifndef MOTEUR3D_H
#define MOTEUR3D_H

#include "../Matrice/Matrice.h"

typedef enum {
    CAM_MODE_ORBIT = 0,
    CAM_MODE_FPS   = 1
} CameraMode;

typedef struct {
    float x, y, z;
} vec3;

typedef struct {
    // --- Common camera state (truth) ---
    CameraMode mode;

    vec3 pos;
    float yaw;    // radians
    float pitch;  // radians
    float roll;   // optional

    vec3 forward;
    vec3 right;
    vec3 up;

    // --- Projection params ---
    float fov_y;
    float aspect;
    float znear;
    float zfar;

    // --- Orbit controller params ---
    vec3 target;
    float distance;

    // --- Matrices (cache) ---
    Matrice view;
    Matrice proj;
    Matrice viewProj;

} Camera;


/**
 * @brief Calculs les données en fonction des valeurs prédéfinie
 * @warning pos, yaw, pitch, roll, fov_y, aspect, znear, zfar nead to be initalized
 */
void updateCameraData(Camera* camera);

#endif //MOTEUR3D_H