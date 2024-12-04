#include "dsp/matrix_functions.h"

void arm_mat_init_f64(
    arm_matrix_instance_f64 *S,
    uint16_t nRows,
    uint16_t nColumns,
    float64_t *pData)
{
    /* Assign Number of Rows */
    S->numRows = nRows;

    /* Assign Number of Columns */
    S->numCols = nColumns;

    /* Assign Data pointer */
    S->pData = pData;
}

/**
  @} end of MatrixInit group
 */