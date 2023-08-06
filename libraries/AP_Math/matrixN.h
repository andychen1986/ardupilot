/*
 *  N dimensional matrix operations
 */

#pragma once

#include "math.h"
#include <stdint.h>
#include "vectorN.h"

template <typename T, uint8_t N>
class VectorN;


template <typename T, uint8_t N>
class MatrixN {
  
    friend class VectorN<T,N>;

public:
    // constructor from zeros
    MatrixN<T,N>(void) {
        memset(v, 0, sizeof(v));        
    }

    // constructor from 4 diagonals
    MatrixN<T,N>(const float d[N]) {
        memset(v, 0, sizeof(v));
        for (uint8_t i = 0; i < N; i++) {
            v[i][i] = d[i];
        }
    }

    MatrixN<T,N>(const T d[][N]) {
        for (uint8_t i = 0; i < N; i++) {
            for (uint8_t j = 0; j < N; j++) {
                v[i][j] = d[i][j];
            }
        }
    }

    // multiply two vectors to give a matrix, in-place
    void mult(const VectorN<T,N> &A, const VectorN<T,N> &B);

    // subtract B from the matrix
    MatrixN<T,N> &operator -=(const MatrixN<T,N> &B);

    // add B to the matrix
    MatrixN<T,N> &operator +=(const MatrixN<T,N> &B);

    // multiply B to the matrix
    VectorN<T,N> operator *(const VectorN<T,N> &B);
    
    // Matrix symmetry routine
    void force_symmetry(void);

private:
    T v[N][N];
};
