//
//  Matrix3x3.hpp
//  Metal Project
//
//  Created by Oliver Homer on 09/11/2026.
//

#pragma once
#include <memory>
#include "Vector3D.h"
#include "Vector4D.h"
#include <array>


constexpr float TWO_PI = 6.28318530718f;

// LEFT HANDED COORDINATE SYSTEM: POSITIVE X = RIGHT, POSITIVE Y = UP, POSITIVE Z = AWAY
// ROTATIONS = ANTICLOCKWISE AS LOOKING ALONG POSITIVE AXIS


class Matrix3x3{
public:
    float mat_[3][3] = {};

    void setIdentity()
    {
        ::memset(mat_, 0, sizeof(float) * 9);
        mat_[0][0] = 1;
        mat_[1][1] = 1;
        mat_[2][2] = 1;
    }

    void setTranslation(const Vector3D& translation)
    {
        mat_[3][0] = translation.x_;
        mat_[3][1] = translation.y_;
        mat_[3][2] = translation.z_;
    }

    void setMatrix(Matrix3x3 matrix)
    {
        ::memcpy(&mat_, &matrix, sizeof(float) * 9);
    }


    void operator *=(const Matrix3x3& matrix)
    {
        Matrix3x3 out;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
            {
                out.mat_[i][j] =
                    mat_[i][0] * matrix.mat_[0][j] + mat_[i][1] * matrix.mat_[1][j] +
                    mat_[i][2] * matrix.mat_[2][j];
            }
        ::memcpy(mat_, out.mat_, sizeof(float) * 9);
    }

    Matrix3x3 operator *(Matrix3x3 matrix)
    {
        Matrix3x3 out;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
            {
                out.mat_[i][j] =
                    mat_[i][0] * matrix.mat_[0][j] + mat_[i][1] * matrix.mat_[1][j] +
                    mat_[i][2] * matrix.mat_[2][j];
            }
        return out;
    }
    
    Matrix3x3 operator *(float x)
    {
        Matrix3x3 out;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
            {
                out.mat_[i][j] =
                mat_[i][j] * x;
            }
        return out;
    }
    
    Matrix3x3 operator+(Matrix3x3 in)
    {
        Matrix3x3 out;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
            {
                out.mat_[i][j] =
                mat_[i][j] + in.mat_[i][j];
            }
        return out;
    
    }
    
    Matrix3x3 operator-(Matrix3x3 in)
    {
        Matrix3x3 out;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
            {
                out.mat_[i][j] =
                mat_[i][j] - in.mat_[i][j];
            }
        return out;
    
    }
    
    Matrix3x3()
    {
        setIdentity();
    }
    
    Matrix3x3(std::array<float,9> values)
    {
        for(int r=0;r<3;r++)
            for(int c=0;c<3;c++)
                mat_[r][c] = values[r*3+c];
    }
    
    Matrix3x3(const Vector3D& column, const Vector3D& row)
    {
        float x[3],y[3];
        x[0]=column.x_;
        x[1]=column.y_;
        x[2]=column.z_;
        y[0]=row.x_;
        y[1]=row.y_;
        y[2]=row.z_;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
            {
                mat_[i][j]=x[i]*y[j];
            }
    }

    void print()
    {
        for(int r=0;r<3;r++)
        {
            for(int c=0;c<3;c++)
            {
                std::cout << mat_[r][c] << " ";
            }
            std::cout<<std::endl;
        }
    }
    
    float getDeterminant()
    {
        Vector3D minor, v1, v2, v3;
        float det;

        v1 = Vector3D(this->mat_[0][0], this->mat_[1][0], this->mat_[2][0]);
        v2 = Vector3D(this->mat_[0][1], this->mat_[1][1], this->mat_[2][1]);
        v3 = Vector3D(this->mat_[0][2], this->mat_[1][2], this->mat_[2][2]);

        det = Vector3D::dot(v1,Vector3D::cross(v2,v3));
                                 
        return det;
    }
    
    float getTrace()
    {
        return mat_[0][0]+mat_[1][1]+mat_[2][2];
    }

    Matrix3x3 invert()
    {
        float det = getDeterminant();
        Matrix3x3 cof;
        cof.mat_[0][0] =  (mat_[1][1] * mat_[2][2])-(mat_[1][2] * mat_[2][1]);
        cof.mat_[0][1] = -((mat_[1][0] * mat_[2][2])-(mat_[1][2] * mat_[2][0]));
        cof.mat_[0][2] =  (mat_[1][0] * mat_[2][1])-(mat_[1][1] * mat_[2][0]);
        cof.mat_[1][0] = -((mat_[0][1] * mat_[2][2])-(mat_[0][2] * mat_[2][1]));
        cof.mat_[1][1] =  (mat_[0][0] * mat_[2][2])-(mat_[0][2] * mat_[2][0]);
        cof.mat_[1][2] = -((mat_[0][0] * mat_[2][1])-(mat_[0][1] * mat_[2][0]));
        cof.mat_[2][0] =  (mat_[0][1] * mat_[1][2])-(mat_[0][2] * mat_[1][1]);
        cof.mat_[2][1] = -((mat_[0][0] * mat_[1][2])-(mat_[0][2] * mat_[1][0]));
        cof.mat_[2][2] =  (mat_[0][0] * mat_[1][1])-(mat_[0][1] * mat_[1][0]);

        Matrix3x3 inv = cof.transpose() * (1/det);
        
        return inv;
    }

    Matrix3x3 transpose() const
    {
        Matrix3x3 out;
        for(int i=0; i<3;i++)
            for(int j=0; j<3;j++)
                out.mat_[i][j] = mat_[j][i];
        return out;
    }

};
