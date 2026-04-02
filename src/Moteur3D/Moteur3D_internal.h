#ifndef MOTEUR3D_INTERNAL_H
#define MOTEUR3D_INTERNAL_H

#include "Moteur3D/Moteur3D.h"
#include "../Matrice/Matrice.h"

typedef struct M3D_CameraInternal {
      Matrice view;
      Matrice proj;
      Matrice viewProj;
} M3D_CameraInternal;

#endif // MOTEUR3D_INTERNAL_H
