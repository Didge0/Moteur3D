#ifndef MATRICE_H
#define MATRICE_H

#include <stdlib.h>
#include <stdbool.h>

typedef struct{
      void* data;
      size_t shape[2];
      size_t strides[2];
      size_t dtype;
}Matrice;

typedef void (*Print_fct_ptr)(const void* elem);
typedef void (*Operator)(void* Dst, const void* val1, const void* val2);

/**
 * @param width: Largeur de la matrice
 * @param height: Hauteur de la matrice
 * @param dtype: Taille en octet du type (ex: char->1octet)
 */
Matrice matrice_init(const size_t width, const size_t height, size_t dtype);

/**
 * @param tab: Pointeur vers un tableau d'élément
 * @param width: Largeur de la matrice
 * @param height: Hauteur de la matrice
 * @param dtype: Taille en octet du type (ex: char->1octet)
 */
Matrice matrice_init_tab(const void* tab, const size_t width, const size_t height, size_t dtype);

Matrice matrice_init_mat(const Matrice* mat);

void matrice_free(Matrice* mat);

void* matrice_get(const Matrice* mat, const size_t idx_w, const size_t idx_h);

void matrice_show(const Matrice* mat, const Print_fct_ptr print);

Matrice matrice_add(const Matrice* mat1, const Matrice* mat2, Operator op);
Matrice matrice_sub(const Matrice* mat1, const Matrice* mat2, Operator op);
Matrice matrice_mul_scal(const Matrice* mat, void* val, size_t dtype);
Matrice matrice_prod_vect(const Matrice* mat1, const Matrice* mat2, Operator op_add, Operator op_mul);




#endif // MATRICE_H
