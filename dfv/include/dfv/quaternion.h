/* Clase Quaternion
 * Incluye operaciones para creación y manipulación
 * de cuaterniones de orientación
 *
 * Autor: Daniel Fernández Villanueva
 */

#ifndef Quaternion_H
#define Quaternion_H

#include <iostream>
#include <cmath>
#include <sstream>
#include <dfv/vector3.h>
#include <dfv/utils.h>
#include <tf/transform_datatypes.h>

//! El espacio de nombres para la librería dfv
namespace dfv
{
    class Vector3;
    
    /*! \brief Clase para la creación y manipulación de cuaterniones.
    */
    class Quaternion
    {
        public:
            
            /*! \brief Constructor de un cuaternión nulo.
            
                Construye un cuaternión cuyas componentes son nulas.
                \f$q=0+0i+0j+0k\f$
            */
            Quaternion();
            
            /*! \brief Constructor de un cuaternión arbitrario.
                
                Construye un cuaternión cuyas componentes son las especificadas.
                \f$q=w+xi+yj+zk\f$
            */
            Quaternion(double w_, double x_, double y_, double z_);
            
            /*! \brief Constructor copia de un cuaternión a partir de un vector.
                
                Construye un cuaternión cuyas componentes imaginarias son las 
                componentes del vector especificado.
                \f$v=x\vec{i}+y\vec{j}+z\vec{k}\rightarrow q=xi+yj+zk\f$
            */
            explicit Quaternion(const Vector3& v);
            
            /*! \brief Destructor.
            */
            ~Quaternion();
            
            /*! \brief Devuelve el negativo del cuaternión.
                
                Operador unario que devuelve un cuaternión cuyas componentes son
                las negativas del cuaternión original.
                Dado \f$q=w+xi+yj+zk\f$ esta función devuelve el cuaternión
                \f$q'=-w-xi-yj-zk\f$
            */
            const Quaternion operator-() const;
            
            // ******** Operador de asignación ******** //
            /*! \brief Asigna un cuaternión a la variable especificada.
                
                Operador binario que asigna a una variable el cuaternión 
                especificado.
            */
            Quaternion& operator=(const Quaternion& q);
            
            // ******** Operadores de asignación compuestos ******** //
            /*! \brief Suma a un cuaternión el cuaternión especificado.
                
                Operador de asignación compuesto. Suma al cuaternión original el 
                cuaternión especificado.
            */
            Quaternion& operator+=(const Quaternion& q);
            
            /*! \brief Resta a un cuaternión el cuaternión especificado.
                
                Operador de asignación compuesto. Resta al cuaternión original el 
                cuaternión especificado.
            */
            Quaternion& operator-=(const Quaternion& q);
            
            /*! \brief Multiplica (Producto de Hamilton) un cuaternión por el cuaternión especificado.
                
                Operador de asignación compuesto. Realiza el producto de 
                Hamilton del cuaternión original por el cuaternión especificado.
            */
            Quaternion& operator*=(const double k);
            
            // ******** Operadores aritméticos binarios ******** //
            /*! \brief Devuelve la suma del cuaternión original y el cuaternión especificado.
                
                Operador de aritmético binario. Devuelve la suma del cuaternión 
                original y el cuaternión especificado. 
                Dado \f$q_1=w_1+x_1i+y_1j+z_1k\f$ y \f$q_2=w_2+x_2i+y_2j+z_2k\f$
                esta función devuelve el cuaternión 
                \f$q'=q_1+q_2=(w_1+w_2)+(x_1+x_2)i+(y_1+y_2)j+(z_1+z_2)k\f$
            */
            const Quaternion operator+(const Quaternion& q) const;
            
            /*! \brief Devuelve la resta del cuaternión original y el cuaternión especificado.
                
                Operador de aritmético binario. Devuelve la resta del cuaternión 
                original y el cuaternión especificado. 
                Dado \f$q_1=w_1+x_1i+y_1j+z_1k\f$ y \f$q_2=w_2+x_2i+y_2j+z_2k\f$
                esta función devuelve el cuaternión 
                \f$q'=q_1+q_2=(w_1-w_2)+(x_1-x_2)i+(y_1-y_2)j+(z_1-z_2)k\f$
            */
            const Quaternion operator-(const Quaternion& q) const;
            
            /*! \brief Devuelve el producto de un numero escalar por el cuaternión original.
                
                Operador de aritmético binario. Devuelve el producto de un número 
                escalar por el cuaternión original. 
                Dado un escalar \f$a\f$ y un cuaternión \f$q=w+xi+yj+zk\f$
                esta función devuelve el cuaternión 
                \f$q'=aq=(aw)+(ax)i+(ay)j+(az)k\f$
            */            
            friend const Quaternion operator*(double k, Quaternion& q);
            
            /*! \brief Devuelve el producto de cuaternión por un escalar.
                
                Operador de aritmético binario. Devuelve el producto de un 
                cuaternión por un número escalar. 
                Dado un cuaternión \f$q=w+xi+yj+zk\f$ y un escalar \f$a\f$
                esta función devuelve el cuaternión 
                \f$q'=qa=(wa)+(xa)i+(ya)j+(za)k\f$
            */
            const Quaternion operator*(double k) const;
            
            /*! \brief Devuelve el producto de Hamilton de dos cuaterniones.
                
                Operador de aritmético binario. Devuelve el producto de Hamilton
                de dos cuaterniones. 
                Dados dos cuaterniones 
                \f$q_1=w_1+x_1i+y_1j+z_1k\f$ y \f$q_2=w_2+x_2i+y_2j+z_2k\f$
                esta función devuelve el cuaternión resultado de realizar
                el producto de Hamilton 
                \f$q'=q_1q_2=(w_1w_2 - x_1x_2 - y_1y_2 - z_1z_2) +
                (x_1w_2 + w_1x_2 - z_1y_2 + y_1z_2)i +
                (y_1w_2 + z_1x_2 + w_1y_2 - x_1z_2)j +
                (z_1w_2 - y_1x_2 + x_1y_2 + w_1z_2)k \f$
            */
            const Quaternion operator*(const Quaternion& q) const;            
            
            // ******** Operadores de comparación ******** //
            
            /*! \brief Devuelve true si los dos cuaterniones son iguales.
                
                Operador de comparación. Devuelve true si las componentes de los
                dos cuaterniones son iguales.
            */
            bool operator==(const Quaternion& q) const;
            
            /*! \brief Devuelve true si los dos cuaterniones son distintos.
                
                Operador de comparación. Devuelve true si hay por lo menos una 
                componente de los dos cuaterniones que es distinta entre sí.
            */
            bool operator!=(const Quaternion& q) const;
            
            /*! \brief Devuelve una cadena de texto con una representación del cuaternión.
            */
            std::string             ToString() const;
            
            /*! \brief Devuelve el módulo del cuaternión.
            
                Función que devuelve el módulo/norma del cuaternión. Dado un 
                cuaternión \f$q=w+xi+yj+zk\f$, el módulo del cuaternión es el
                resultado de la operación: \f$||q||=\sqrt{w^2+x^2+y^2+z^2}\f$
            */
            double                  GetModulus() const;
            
            // Función que normaliza el cuaternión:
            void                    Normalize();
            
            // Función que devuelve el conjugado del cuaternión:
            const Quaternion        GetConjugate() const;
            
            // Función que devuelve el cuaternión de rotación
            // definido por el eje [axisx_, axisy_, axisz_]
            // y el ángulo angle_:
            static const Quaternion GetRotationQuaternion(const double axisx_, 
                                                          const double axisy_, 
                                                          const double axisz_, 
                                                          const double angle_);
            
            // Función que devuelve el cuaternión de rotación
            // definido por el eje axis_
            // y el ángulo angle_:                                              
            static const Quaternion GetRotationQuaternion(const Vector3& axis_, 
                                                          const double angle_);
                                                          
            // Función que devuelve el cuaternión de rotación
            // definido por el eje perpendicular a v_before y v_after:
            static const Quaternion GetRotationQuaternion(const Vector3& v_before, 
                                                          const Vector3& v_after);
                                                          
            // Función que devuelve el cuaternión de rotación
            // del conjunto de vectores v1 y v2 
            // Se supone que v1 y v2 son perpendiculares entre sí:
            static const Quaternion GetRotationQuaternion(const Vector3& v1_before,
                                                          const Vector3& v1_after,
                                                          const Vector3& v2_before,
                                                          const Vector3& v2_after);

            
            // Función que descompone el cuaternión en otros tres cuaterniones
            // definidos por los ejes v1, v2 y v3, y unos valores iniciales
            // de los ángulos angle_1, angle_2 y angle_3
            // Los valores finales de angle_1, angle_2 y angle_3 son
            // los resultados obtenidos para los ángulos asociados
            // a los cuaterniones.
            // Los vectores v1, v2 y v3 tienen que ser linearmente
            // independientes para asegurar la obtención de un resultado
            // correcto.
            void                    Decompose(double& angle_1,
                                              double& angle_2,
                                              double& angle_3,
                                              const Vector3& v1,
                                              const Vector3& v2,
                                              const Vector3& v3) const;

            // Función para obtener el eje y el ángulo asociados al
            // cuaternión:
            void                    GetAxisAndAngle(Vector3& vector, 
                                                    double& angle) const;
                                                    
            // Función para obtener el roll, pitch y yaw del cuaternión
            void                    GetRPY(double& roll, double& pitch, double& yaw, unsigned int solution = 1);
            
            // Función que devuelve el cuaternión de diferencia entre dos cuaterniones
            static const Quaternion GetDifference(const Quaternion& q1, const Quaternion& q2);

            // Quaterniones unitarios colineales a las componentes w, x, y, z:
            static const Quaternion identity;
            static const Quaternion i;
            static const Quaternion j;
            static const Quaternion k;
            
            // Componentes del cuaternión:
            double w;
            double x;
            double y;
            double z;
            
        protected:
        private:

            
    };
    
    std::ostream& operator<<(std::ostream& os, const Quaternion& q);

}

#endif // Quaternion_H
