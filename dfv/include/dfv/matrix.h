/* Clase Matrix
 * Incluye operaciones para creación y manipulación
 * de matrices de cualquier dimensión
 *
 * Autor: Daniel Fernández Villanueva
 */

#ifndef MATRIX_H
#define MATRIX_H

#include <iostream>
#include <vector>
#include <sstream>
#include <cstdlib>

//! El espacio de nombres para la librería dfv
namespace dfv
{
    /*! \brief Clase para la creación y manipulación de matrices de dimensión arbitraria.
    */
    class Matrix
    {
        public:
            Matrix();
            Matrix(unsigned int size);
            Matrix(unsigned int rows, unsigned int columns);
            virtual ~Matrix();
            
            // ******** Operador de asignación ******** //
            Matrix& operator=(const Matrix& q);
            
            // ******** Operadores de asignación compuestos ******** //
            Matrix& operator+=(const Matrix& q);
            Matrix& operator-=(const Matrix& q);
            Matrix& operator*=(const double k);
            
            // ******** Operadores aritméticos binarios ******** //
            const Matrix operator+(const Matrix& q) const;
            const Matrix operator-(const Matrix& q) const;            
            friend const Matrix operator*(double k, Matrix& q);
            const Matrix operator*(double k) const;
            // Producto de Matrices:
            const Matrix operator*(const Matrix& q) const;            
            
            // ******** Operadores de comparación ******** //
            bool operator==(const Matrix& q) const;
            bool operator!=(const Matrix& q) const;
            
            // Función que devuelve una representación
            // de la matriz como texto:
            std::string             ToString() const;
            
            // Función que crea una matriz de rows filas y columns columnas
            Matrix&             Create(unsigned int rows, unsigned int columns, double value = 0);
            
            // Función que devuelve el elemento situado en la fila row y columna col
            double              Get(unsigned int row, unsigned int col) const;
            
            // Función que permie cambiar el valor del elemento situado en la fila row y columna col
            void                Set(unsigned int row, unsigned int col, double value);
            
            // Función que devuelve el número de filas de la matriz
            unsigned int        GetRows() const;
            
            // Función que devuelve el número de columnas de la matriz
            unsigned int        GetColumns() const;
            
            // Función que devuleve el menor asociado a la fila row y columna column
            Matrix              GetMinor(unsigned int row, unsigned int column) const;
            
            // Función que asigna un valor aleatoria ontre 0 y 1 a cada elemento de la matriz
            void                Randomize();
            
            // Función que devuelve el determinante de la matriz
            double              GetDeterminant() const;
            
            // Función que devuelve la traspuesta de la matriz
            const Matrix        GetTransposed() const;
            
            // Función que devuelve el adjunto de la matriz
            const Matrix        GetAdjoint() const;
            
            // Función que devuelve la matriz transpuesta conjugada
            const Matrix        GetAdjugate() const;
            
            // Función que devuelve la inversa de la matriz
            const Matrix        GetInverse() const;
            
            // Operador equivalente a la función Get()
            double              operator()(unsigned int row, unsigned int column) const;
            
            // Función que devuelve una matriz unitaria con la dimensión dada
            static const Matrix Identity(unsigned int size);

        protected:
        private:
            unsigned int        rows;
            unsigned int        columns;
            std::vector<double> data;
    };

    std::ostream& operator<<(std::ostream& os, Matrix& m);

}

#endif // Matrix_H
