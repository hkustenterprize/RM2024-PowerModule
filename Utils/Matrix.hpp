/**
 ******************************************************************************
 * @file    matrix.cpp/h
 * @brief   Matrix/vector calculation. 矩阵/向量运算
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "AppConfig.h"
#include "EnableIf.hpp"
#include "arm_math.h"
#pragma once

template <int _rows, int _cols>
class Matrixf
{
   public:
    /**
     * @brief Constructor without input data
     * @param
     */
    constexpr Matrixf(void) : rows_(_rows), cols_(_cols) { arm_mat_init_f32(&arm_mat_, _rows, _cols, this->data_); }

    /**
     * @brief Constructor with input data
     * @param data    A 2-D array buffer that stores the data
     */
    constexpr Matrixf(float data[_rows * _cols]) : Matrixf()
    {
        memcpy(this->data_, data, _rows * _cols * sizeof(float));
        arm_mat_init_f32(&arm_mat_, _rows, _cols, this->data_);
    }

    /**
     * @brief      Copy Constructor
     * @param mat  The copied matrix
     */
    constexpr Matrixf(const Matrixf<_rows, _cols> &mat) : Matrixf()
    {
        memcpy(this->data_, mat.data_, _rows * _cols * sizeof(float));
        arm_mat_init_f32(&arm_mat_, _rows, _cols, this->data_);
    }

    /**
     * @brief Destructor
     */
    ~Matrixf(void) {}

    /**
     * @brief returns the row size of the matrix
     * @return _rows  The row size of the matrix
     */
    uint32_t rows(void) const { return _rows; }

    /**
     * @brief return the column size of the matrix
     * @return _cols  The column size of the matrix
     */
    uint32_t cols(void) const { return _cols; }

    /**
     * @brief Return the element of the matrix
     * @param row  The row
     */
    float *operator[](const int &row) { return &this->data_[row * _cols]; }

    /**
     * @brief Copy assignment of the matrix(row * size) instance
     * @param mat   The copied prototype
     * @return      *this matrix
     */
    Matrixf<_rows, _cols> &operator=(const Matrixf<_rows, _cols> mat)
    {
        memcpy(this->data_, mat.data_, _rows * _cols * sizeof(float));
        return *this;
    }

    /**
     * @brief      Additional operator of two matrices(row * size)
     * @param mat  The matrix on the right hand side
     * @note  This function returns itself as the result
     * @return     The sum of two matrices
     */
    Matrixf<_rows, _cols> &operator+=(const Matrixf<_rows, _cols> mat)
    {
        arm_mat_add_f32(&this->arm_mat_, &mat.arm_mat_, &this->arm_mat_);
        return *this;
    }

    /**
     * @brief Substraction operator of two matrices(row * size)
     * @param mat The matrix on the left hand side
     * @note  This function returns itself as the result
     * @return    The difference of two matrices
     */
    Matrixf<_rows, _cols> &operator-=(const Matrixf<_rows, _cols> mat)
    {
        arm_mat_sub_f32(&this->arm_mat_, &mat.arm_mat_, &this->arm_mat_);
        return *this;
    }

    /**
     * @brief Scalar operator of the matrix and a scaling factor
     * @param val The scaling factor
     * @note  This function returns itself as the result
     * @return    THe scaled matrix
     */
    Matrixf<_rows, _cols> &operator*=(const float &val)
    {
        arm_mat_scale_f32(&this->arm_mat_, val, &this->arm_mat_);
        return *this;
    }

    /**
     * @brief Scalar operator of the matrix and a division factor
     * @param val The division factor
     * @note  This function returns itself as the result
     * @retval    matrix / val
     * @return    The scaled matrix
     */
    Matrixf<_rows, _cols> &operator/=(const float &val)
    {
        arm_mat_scale_f32(&this->arm_mat_, 1.f / val, &this->arm_mat_);
        return *this;
    }

    /**
     * @brief Additonal operator
     * @note This function doesn't return itself but instead a new matrix instance
     * @param mat The matrix on the right hand side
     * @return The sum of the additional matrix
     */
    Matrixf<_rows, _cols> operator+(const Matrixf<_rows, _cols> &mat) const
    {
        Matrixf<_rows, _cols> res;
        arm_mat_add_f32(&this->arm_mat_, &mat.arm_mat_, &res.arm_mat_);
        return res;
    }

    /**
     * @brief Substraction matrix
     * @note This function does not return itself but instead a new matrix instance
     * @param mat matrix on the right hand side
     * @return The sum of the substracted matrix
     */
    Matrixf<_rows, _cols> operator-(const Matrixf<_rows, _cols> &mat) const
    {
        Matrixf<_rows, _cols> res;
        arm_mat_sub_f32(&this->arm_mat_, &mat.arm_mat_, &res.arm_mat_);
        return res;
    }

    /**
     * @brief Scalar operator of the matrix and a scaling factor
     * @param val The scaling factor
     * @note      This function does not return itself
     * @return    THe scaled matrix
     */
    Matrixf<_rows, _cols> operator*(const float &val) const
    {
        Matrixf<_rows, _cols> res;
        arm_mat_scale_f32(&this->arm_mat_, val, &res.arm_mat_);
        return res;
    }

    /**
     * @brief Scalar operator of the matrix and a scaling factor
     * @param val The scaling factor on the left hand side
     * @note      This function does not return itself
     * @note      This time the scaling factor is on the left hand side
     * @return    THe scaled matrix
     */
    friend Matrixf<_rows, _cols> operator*(const float &val, const Matrixf<_rows, _cols> &mat)
    {
        arm_status s;
        Matrixf<_rows, _cols> res;
        s = arm_mat_scale_f32(&mat.arm_mat_, val, &res.arm_mat_);
        return res;
    }

    /**
     * @brief Scalar operator of the matrix and a division factor
     * @param val The division factor
     * @note  This function returns itself as the result
     * @retval    matrix / val
     * @return    The scaled matrix
     */
    Matrixf<_rows, _cols> operator/(const float &val) const
    {
        Matrixf<_rows, _cols> res;
        arm_mat_scale_f32(&this->arm_mat_, 1.f / val, &res.arm_mat_);
        return res;
    }

    /**
     * @brief The matrix multiplication
     * @param mat1 the matrix on the LHS
     * @param mat2 the matrix on the RHS
     * @return The multiplication result
     */
    template <int cols2>
    friend Matrixf<_rows, cols2> operator*(const Matrixf<_rows, _cols> &mat1, const Matrixf<_cols, cols2> &mat2)
    {
        Matrixf<_rows, cols2> res;
        arm_mat_mult_f32(&mat1.arm_mat_, &mat2.arm_mat_, &res.arm_mat_);
        return res;
    }

    /**
     * @brief Compare whether two matrices are identical
     *
     */
    bool operator==(const Matrixf<_rows, _cols> &mat) const
    {
        for (int i = 0; i < _rows * _cols; i++)
        {
            if (this->data_[i] != mat.data_[i])
                return false;
        }
        return true;
    }

    // Submatrix
    template <int rows, int cols>
    Matrixf<rows, cols> block(const int &start_row, const int &start_col) const
    {
        Matrixf<rows, cols> res;
        for (int row = start_row; row < start_row + rows; row++)
        {
            memcpy((float *)res[0] + (row - start_row) * cols, (float *)this->data_ + row * _cols + start_col, cols * sizeof(float));
        }
        return res;
    }

    /**
     * @brief Return the specific row of the matrix
     * @param row The row index
     * @retval The row vector presented in the matrix from
     */
    Matrixf<1, _cols> row(const int &row) const { return block<1, _cols>(row, 0); }

    /**
     * @brief Return the specific row of the matrix
     * @param col The column index
     * @retval The column vector presented in the matrix from
     */
    Matrixf<_rows, 1> col(const int &col) const { return block<_rows, 1>(0, col); }

    /**
     * @brief Get the transpose of the matrix
     * @param
     * @retval the transposed matrix
     */
    Matrixf<_cols, _rows> trans(void) const
    {
        Matrixf<_cols, _rows> res;
        arm_mat_trans_f32(&arm_mat_, &res.arm_mat_);
        return res;
    }
    // Trace

    /**
     * @brief Get the trace of the matrix
     * @param
     * @retval The trace of the matrix
     */
    float trace(void) const
    {
        float res = 0;
        for (int i = 0; i < fmin(_rows, _cols); i++)
        {
            res += (*this)[i][i];
        }
        return res;
    }

    /**
     * @brief Get the norm of the matrix
     * @param
     * @retval The norm of the matrix
     */
    float norm(void) const { return sqrtf((this->trans() * *this)[0][0]); }

    /**
     * @brief Get the inverse of the matrix
     * @param
     * @retval The inverse of the matrix
     */
    Matrixf<_cols, _rows> inv(void) const
    {
        if (_cols != _rows)
            return Matrixf<_cols, _rows>::zeros();

        Matrixf<_cols, _rows> res;
        arm_status status = arm_mat_inverse_f32(&this->arm_mat_, &res);

        if (status == ARM_MATH_SINGULAR)
            return Matrixf<_cols, _rows>::zeros();

        return res;
    }

    /*==============================================================*/
    // Static function
    /**
     * @brief Returns a _rows x _cols zero matrix
     * @tparam _rows The row size
     * @tparam _cols The column size
     * @retval The zero matrix
     */
    static Matrixf<_rows, _cols> zeros(void)
    {
        float data[_rows * _cols] = {0};
        return Matrixf<_rows, _cols>(data);
    }

    /**
     * @brief Returns a _rows x _cols one matrix
     * @tparam _rows The row size
     * @tparam _cols The column size
     * @retval The one matrix
     */
    static Matrixf<_rows, _cols> ones(void)
    {
        float data[_rows * _cols] = {0};
        for (int i = 0; i < _rows * _cols; i++)
        {
            data[i] = 1;
        }
        return Matrixf<_rows, _cols>(data);
    }

    /**
     * @brief Returns a _rows * columns  matrix
     * @tparam _rows The row size
     * @tparam _cols The column size
     * @retval The identity matrix
     */
    static Matrixf<_rows, _cols> eye(void)
    {
        float data[_rows * _cols] = {0};
        for (int i = 0; i < fmin(_rows, _cols); i++)
        {
            data[i * _cols + i] = 1;
        }
        return Matrixf<_rows, _cols>(data);
    }

    /**
     * @brief Returns a _rows x _cols diagonal matrix
     * @tparam _rows The row size
     * @tparam _cols The column size
     * @param vec The diagnoal entries
     * @retval The diagnoanl matrix
     */
    static Matrixf<_rows, _cols> diag(Matrixf<_rows, 1> vec)
    {
        Matrixf<_rows, _cols> res = Matrixf<_rows, _cols>::zeros();
        for (int i = 0; i < fmin(_rows, _cols); i++)
        {
            res[i][i] = vec[i][0];
        }
        return res;
    }

   public:
    arm_matrix_instance_f32 arm_mat_;  // The arm math instance

   protected:
    // The size
    int rows_, cols_;
    // Data buffer
    float data_[_rows * _cols];
};

// namespace matrixf
