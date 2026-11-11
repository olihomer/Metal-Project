//
//  Matrix3x3d.hpp
//  Metal Project
//
//  Created by Oliver Homer on 10/11/2026.
//

#pragma once
#include <memory>
#include "Vector3D.h"
#include "Vector3Dd.hpp"
#include "Vector4D.h"
#include <array>


// LEFT HANDED COORDINATE SYSTEM: POSITIVE X = RIGHT, POSITIVE Y = UP, POSITIVE Z = AWAY
// ROTATIONS = ANTICLOCKWISE AS LOOKING ALONG POSITIVE AXIS


class Matrix3x3d{
public:
    double mat_[3][3] = {};

    void setIdentity()
    {
        ::memset(mat_, 0, sizeof(double) * 9);
        mat_[0][0] = 1;
        mat_[1][1] = 1;
        mat_[2][2] = 1;
    }


    void setMatrix(Matrix3x3d matrix)
    {
        ::memcpy(&mat_, &matrix, sizeof(double) * 9);
    }


    void operator *=(const Matrix3x3d& matrix)
    {
        Matrix3x3d out;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
            {
                out.mat_[i][j] =
                    mat_[i][0] * matrix.mat_[0][j] + mat_[i][1] * matrix.mat_[1][j] +
                    mat_[i][2] * matrix.mat_[2][j];
            }
        ::memcpy(mat_, out.mat_, sizeof(double) * 9);
    }

    Matrix3x3d operator *(Matrix3x3d matrix)
    {
        Matrix3x3d out;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
            {
                out.mat_[i][j] =
                    mat_[i][0] * matrix.mat_[0][j] + mat_[i][1] * matrix.mat_[1][j] +
                    mat_[i][2] * matrix.mat_[2][j];
            }
        return out;
    }
    
    Matrix3x3d operator *(double x)
    {
        Matrix3x3d out;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
            {
                out.mat_[i][j] =
                mat_[i][j] * x;
            }
        return out;
    }
    
    Vector3Dd operator *(Vector3Dd& vector)
    {
        Vector3Dd out;
        out.x_ = mat_[0][0] * vector.x_ + mat_[0][1] * vector.y_ + mat_[0][2] * vector.z_;
        out.y_ = mat_[1][0] * vector.x_ + mat_[1][1] * vector.y_ + mat_[1][2] * vector.z_;
        out.z_ = mat_[2][0] * vector.x_ + mat_[2][1] * vector.y_ + mat_[2][2] * vector.z_;
        return out;
    }
    
    Matrix3x3d operator+(Matrix3x3d in)
    {
        Matrix3x3d out;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
            {
                out.mat_[i][j] =
                mat_[i][j] + in.mat_[i][j];
            }
        return out;
    
    }
    
    Matrix3x3d operator-(Matrix3x3d in)
    {
        Matrix3x3d out;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
            {
                out.mat_[i][j] =
                mat_[i][j] - in.mat_[i][j];
            }
        return out;
    
    }
    
    Matrix3x3d()
    {
        setIdentity();
    }
    
    Matrix3x3d(std::array<double,9> values)
    {
        for(int r=0;r<3;r++)
            for(int c=0;c<3;c++)
                mat_[r][c] = values[r*3+c];
    }
    
    Matrix3x3d(const Vector3Dd& column, const Vector3Dd& row)
    {
        double x[3],y[3];
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
    
    double getDeterminant()
    {
        Vector3D minor, v1, v2, v3;
        double det;

        v1 = Vector3D(this->mat_[0][0], this->mat_[1][0], this->mat_[2][0]);
        v2 = Vector3D(this->mat_[0][1], this->mat_[1][1], this->mat_[2][1]);
        v3 = Vector3D(this->mat_[0][2], this->mat_[1][2], this->mat_[2][2]);

        det = Vector3D::dot(v1,Vector3D::cross(v2,v3));
                                 
        return det;
    }
    
    double getTrace()
    {
        return mat_[0][0]+mat_[1][1]+mat_[2][2];
    }

    Matrix3x3d invert()
    {
        double det = getDeterminant();
        Matrix3x3d cof;
        cof.mat_[0][0] =  (mat_[1][1] * mat_[2][2])-(mat_[1][2] * mat_[2][1]);
        cof.mat_[0][1] = -((mat_[1][0] * mat_[2][2])-(mat_[1][2] * mat_[2][0]));
        cof.mat_[0][2] =  (mat_[1][0] * mat_[2][1])-(mat_[1][1] * mat_[2][0]);
        cof.mat_[1][0] = -((mat_[0][1] * mat_[2][2])-(mat_[0][2] * mat_[2][1]));
        cof.mat_[1][1] =  (mat_[0][0] * mat_[2][2])-(mat_[0][2] * mat_[2][0]);
        cof.mat_[1][2] = -((mat_[0][0] * mat_[2][1])-(mat_[0][1] * mat_[2][0]));
        cof.mat_[2][0] =  (mat_[0][1] * mat_[1][2])-(mat_[0][2] * mat_[1][1]);
        cof.mat_[2][1] = -((mat_[0][0] * mat_[1][2])-(mat_[0][2] * mat_[1][0]));
        cof.mat_[2][2] =  (mat_[0][0] * mat_[1][1])-(mat_[0][1] * mat_[1][0]);

        Matrix3x3d inv = cof.transpose() * (1/det);
        
        return inv;
    }

    Matrix3x3d transpose() const
    {
        Matrix3x3d out;
        for(int i=0; i<3;i++)
            for(int j=0; j<3;j++)
                out.mat_[i][j] = mat_[j][i];
        return out;
    }

};
