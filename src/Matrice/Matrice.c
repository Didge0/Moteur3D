#include "Matrice.h"
#include <string.h>


Matrice matrice_init(const size_t width, const size_t height, size_t dtype){
      Matrice mat;
      if(width==0 || height==0){
            perror("[initialisation]: Width or Height equal zero");
      }
      mat.data = malloc(width*height*dtype);
      mat.shape[0] = width;
      mat.shape[1] = height;
      mat.strides[0] = 1;
      mat.strides[1] = width;
      mat.dtype = dtype;
      return mat;
}

Matrice matrice_init_val(const int val, const size_t width, const size_t height, size_t dtype){
      Matrice mat;
      if(width==0 || height==0){
            perror("[initialisation]: Width or Height equal zero");
      }
      mat.data = malloc(width*height*dtype);
      memset(mat.data, val, width*height*dtype);
      mat.shape[0] = width;
      mat.shape[1] = height;
      mat.strides[0] = 1;
      mat.strides[1] = width;
      mat.dtype = dtype;
      return mat;
}

Matrice matrice_init_tab(const void* tab, const size_t width, const size_t height, size_t dtype){
      Matrice mat = matrice_init(width, height, dtype);
      memcpy(mat.data, tab, width*height*dtype);
      return mat;
}

Matrice matrice_init_mat(const Matrice* mat){
      Matrice new_mat = matrice_init(mat->shape[0], mat->shape[1], mat->dtype);
      memcpy(new_mat.data, mat->data, mat->shape[0]*mat->shape[1]*mat->dtype);
      return new_mat;
}

void matrice_free(Matrice* mat){
     free( mat->data);
     mat->data = NULL;
}

void* matrice_get(const Matrice* mat, const size_t idx_w, const size_t idx_h){
      if(idx_w >= mat->shape[0] || idx_h>=mat->shape[1]){
            perror("OOB: idx_w or idw_h is too big");
            return NULL;
      }
      return (char*)mat->data + (idx_h * mat->strides[1]*mat->dtype) + (idx_w*mat->strides[0]*mat->dtype);
}

void matrice_show(const Matrice* mat, const Print_fct_ptr print){
      void* val_to_print;
      for(size_t i=0; i<mat->shape[1]; i++){
            printf("|");
            for(size_t j=0; j<mat->shape[0]; j++){
                  if(j!=0)
                        printf(",");
                  val_to_print = matrice_get(mat, j, i);
                  print(val_to_print);
                  
            }
            printf("|\n");
      }
}

Matrice matrice_add(const Matrice* mat1, const Matrice* mat2, Operator op){
      if(mat1->dtype != mat2->dtype){
            perror("[matrice_add] - dtype different\n");
      }
      if(mat1->shape[0] != mat2->shape[0]){
            perror("[matrice_add] - width different\n");
      }
      if(mat1->shape[1] != mat2->shape[1]){
            perror("[matrice_add] - height different\n");
      }
      Matrice mat = matrice_init_mat(mat1);
      for(size_t i=0; i<mat1->shape[0]; i++){
            for(size_t j=0; j<mat1->shape[1]; j++){
                  op(matrice_get(&mat,j,i), matrice_get(&mat,j,i),matrice_get(mat2,j,i));
                  //*(char*)matrice_get(&mat,j,i) += *(char*)matrice_get(mat2,j,i);
            }
      }
      return mat;
}

Matrice matrice_sub(const Matrice* mat1, const Matrice* mat2, Operator op){
      if(mat1->dtype != mat2->dtype){
            perror("[matrice_sub] - dtype different\n");
      }
      if(mat1->shape[0] != mat2->shape[0]){
            perror("[matrice_sub] - width different\n");
      }
      if(mat1->shape[1] != mat2->shape[0]){
            perror("[matrice_sub] - height different\n");
      }
      Matrice mat = matrice_init_mat(mat1);
      for(size_t i=0; i<mat1->shape[0]; i++){
            for(size_t j=0; j<mat1->shape[1]; j++){
                  op(matrice_get(&mat,j,i), matrice_get(&mat,j,i),matrice_get(mat2,j,i));
                  //*(char*)matrice_get(&mat,j,i) -= *(char*)matrice_get(mat2,j,i);
            }
      }
      return mat;
}

Matrice matrice_mul_scal(const Matrice* mat, void* val, size_t dtype){
      if(mat->dtype != dtype){
            perror("[matrice_maul_scal] - dtype different\n");
      }
      Matrice new_mat = matrice_init_mat(mat);
      for(size_t i=0; i<mat->shape[0]; i++){
            for(size_t j=0; j<mat->shape[1]; j++){
                  *(char*)matrice_get(&new_mat,j,i) *= *(char*)val;
            }
      }
      return new_mat;
}

Matrice matrice_prod(const Matrice* mat1, const Matrice* mat2, Operator op_add, Operator op_mul){
      if(mat1->dtype != mat2->dtype){
            perror("[matrice_prod] - dtype different\n");
      }  
      if(mat1->shape[0] != mat2->shape[1]){
            perror("[matrice_prod] - width mat1 different of height mat2\n");
      }
      size_t dx,dy,i;
      Matrice mat = matrice_init_val(0,mat2->shape[0], mat1->shape[1], mat1->dtype);
      void* val = malloc(mat1->dtype);
      for(dx=0; dx<mat.shape[0]; dx++){
            for(dy=0; dy<mat.shape[1]; dy++){
                  *(char*)val = 0;
                  for(i=0; i<mat1->shape[0]; i++){
                        op_mul(val,matrice_get(mat1,i,dy),matrice_get(mat2,dx,i));
                        op_add(matrice_get(&mat,dx,dy),matrice_get(&mat,dx,dy),val);
                  }
            }
      }
      free(val);
      return mat;
}

Matrice vect_prod(const Matrice* mat1, const Matrice* mat2){
      if(mat1->dtype != mat2->dtype){
            perror("[vect_prod] - dtype different\n");
      }
      if(mat1->shape[0] != mat2->shape[0] ||mat1->shape[1] != mat2->shape[1]){
            perror("[vect_prod] - shape different\n");
      }
      if(mat1->shape[0] != 1 || mat1->shape[1] != 0){
            
      }
}


