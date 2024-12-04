#include "dsp/matrix_functions.h"

arm_status arm_mat_add_f64(
    const arm_matrix_instance_f64 *pSrcA,
    const arm_matrix_instance_f64 *pSrcB,
    arm_matrix_instance_f64 *pDst)
{
    arm_status status;
    /* Check for matrix mismatch condition */
    if ((pSrcA->numRows != pSrcB->numRows) ||
        (pSrcA->numCols != pSrcB->numCols) ||
        (pSrcA->numRows != pDst->numRows) || (pSrcA->numCols != pDst->numCols))
    {
        /* Set status as ARM_MATH_SIZE_MISMATCH */
        status = ARM_MATH_SIZE_MISMATCH;
    }
    else
    {
        for (int i = 0; i < pSrcA->numRows * pSrcA->numCols; i++)
        {
            pDst->pData[i] = pSrcA->pData[i] + pSrcB->pData[i];
        }
        status = ARM_MATH_SUCCESS;
    }
    return status;
}