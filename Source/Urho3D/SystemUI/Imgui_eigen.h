#pragma once

#include "eigen/Eigen/Dense"
#include "../SystemUI/SystemUI.h"

namespace ImGui {

    void EIGEN_MATRIX_XD_TABLE(const char* str_id, const Eigen::MatrixXd& matrix)
    {
        if (ImGui::BeginTable(str_id, matrix.cols()))
        {
            for (int row = 0; row < matrix.rows(); row++)
            {
                ImGui::TableNextRow();
                for (int column = 0; column < matrix.cols(); column++)
                {
                    ImGui::TableSetColumnIndex(column);
                    ImGui::Text("%f", matrix(row, column));
                }
            }
            ImGui::EndTable();
        }
    }



    void EIGEN_MATRIX_XCD_TABLE(const char* str_id, const Eigen::MatrixXcd& matrix)
    {
        if (ImGui::BeginTable(str_id, matrix.cols()))
        {
            for (int row = 0; row < matrix.rows(); row++)
            {
                ImGui::TableNextRow();
                for (int column = 0; column < matrix.cols(); column++)
                {
                    ImGui::TableSetColumnIndex(column);
                    ImGui::Text("%f + %fj", matrix(row, column).real(), matrix(row, column).imag());
                }
            }
            ImGui::EndTable();
        }
    }
    void EIGEN_VECTOR_XCD_TABLE(const char* str_id, const Eigen::VectorXcd& matrix)
    {
        if (ImGui::BeginTable(str_id, matrix.cols()))
        {
            for (int row = 0; row < matrix.rows(); row++)
            {
                ImGui::TableNextRow();
                for (int column = 0; column < matrix.cols(); column++)
                {
                    ImGui::TableSetColumnIndex(column);
                    ImGui::Text("%f + %fj", matrix(row, column).real(), matrix(row, column).imag());
                }
            }
            ImGui::EndTable();
        }
    }
}





