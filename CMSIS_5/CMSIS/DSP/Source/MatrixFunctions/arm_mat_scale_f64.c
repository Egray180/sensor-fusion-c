#include "dsp/matrix_functions.h"

arm_status arm_mat_scale_f64(
    const arm_matrix_instance_f64 *pSrc,
    float64_t scale,
    arm_matrix_instance_f64 *pDst)
{
    arm_status status;
    /* Check for matrix mismatch condition */
    if ((pSrc->numRows != pDst->numRows) || (pSrc->numCols != pDst->numCols))
    {
        /* Set status as ARM_MATH_SIZE_MISMATCH */
        status = ARM_MATH_SIZE_MISMATCH;
    }
    else
    {
        for (int i = 0; i < pSrc->numRows * pSrc->numCols; i++)
        {
            pDst->pData[i] = scale * pSrc->pData[i];
        }
        status = ARM_MATH_SUCCESS;
    }
    return status;
}