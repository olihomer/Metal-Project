//
//  Matrix2x2.hpp
//  Metal Project
//
//  Created by Oliver Homer on 23/11/2026.
//


#pragma once
#include "Vector2D.h"
#include <memory>
#include <array>


// LEFT HANDED COORDINATE SYSTEM: POSITIVE X = RIGHT, POSITIVE Y = UP, POSITIVE Z = AWAY
// ROTATIONS = ANTICLOCKWISE AS LOOKING ALONG POSITIVE AXIS


class Matrix2x2{
public:
    float mat_[2][2] = {};

    void setIdentity()
    {
        ::memset(mat_, 0, sizeof(float) * 4);
        mat_[0][0] = 1;
        mat_[1][1] = 1;
    }

    void setMatrix(Matrix2x2 matrix)
    {
        ::memcpy(&mat_, &matrix, sizeof(float) * 4);
    }


    void operator *=(const Matrix2x2& matrix)
    {
        Matrix2x2 out;
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
            {
                out.mat_[i][j] =
                mat_[i][0] * matrix.mat_[0][j] + mat_[i][1] * matrix.mat_[1][j];
            }
        ::memcpy(mat_, out.mat_, sizeof(float) * 4);
    }

    Matrix2x2 operator *(Matrix2x2 matrix)
    {
        Matrix2x2 out;
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
            {
                out.mat_[i][j] =
                mat_[i][0] * matrix.mat_[0][j] + mat_[i][1] * matrix.mat_[1][j];
            }
        return out;
    }
    
    Matrix2x2 operator *(float x)
    {
        Matrix2x2 out;
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
            {
                out.mat_[i][j] =
                mat_[i][j] * x;
            }
        return out;
    }
    
    Matrix2x2 operator+(Matrix2x2 in)
    {
        Matrix2x2 out;
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
            {
                out.mat_[i][j] =
                mat_[i][j] + in.mat_[i][j];
            }
        return out;
    
    }
    
    Matrix2x2 operator-(Matrix2x2 in)
    {
        Matrix2x2 out;
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
            {
                out.mat_[i][j] =
                mat_[i][j] - in.mat_[i][j];
            }
        return out;
    
    }
    
    Matrix2x2()
    {
        setIdentity();
    }

    void print(std::string comment)
    {
        std::cout << comment << ":" << std::endl;
        for(int r=0;r<2;r++)
        {
            for(int c=0;c<2;c++)
            {
                std::cout << mat_[r][c] << " ";
            }
            std::cout<<std::endl;
        }
    }
    
    float getDeterminant()
    {
        return ((mat_[0][0]*mat_[1][1])-(mat_[0][1]*mat_[1][0]));
    }
    


    Matrix2x2 getInverse()
    {
        float det = 1/getDeterminant();
        Matrix2x2 inv{};
        inv.mat_[0][0] = mat_[1][1]*det;
        inv.mat_[0][1] = mat_[0][1]*-det;
        inv.mat_[1][0] = mat_[1][0]*-det;
        inv.mat_[1][1] = mat_[0][0]*det;
        
        return inv;
    }

    Matrix2x2 transpose() const
    {
        Matrix2x2 out;
        for(int i=0; i<2;i++)
            for(int j=0; j<2;j++)
                out.mat_[i][j] = mat_[j][i];
        return out;
    }

};


inline Vector2D operator*(const Vector2D& vector, const Matrix2x2& matrix)
{
    Vector2D out{};

    out.x_ = matrix.mat_[0][0] * vector.x_ + matrix.mat_[1][0] * vector.y_;

    out.y_ = matrix.mat_[0][1] * vector.x_ + matrix.mat_[1][1] * vector.y_;

    return out;
}
