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

namespace dfv
{
    class Vector3;

    class Quaternion
    {
        public:

            Quaternion();
            Quaternion(double w_, double x_, double y_, double z_);
            explicit Quaternion(const Vector3& v);
            ~Quaternion();
            
            // ******** Operador de asignación ******** //
            Quaternion& operator=(const Quaternion& q);
            
            // ******** Operadores de asignación compuestos ******** //
            Quaternion& operator+=(const Quaternion& q);
            Quaternion& operator-=(const Quaternion& q);
            Quaternion& operator*=(const double k);
            
            // ******** Operadores aritméticos binarios ******** //
            const Quaternion operator+(const Quaternion& q) const;
            const Quaternion operator-(const Quaternion& q) const;            
            friend const Quaternion operator*(double k, Quaternion& q);
            const Quaternion operator*(double k) const;
            // Producto de Hamilton:
            const Quaternion operator*(const Quaternion& q) const;            
            
            // ******** Operadores de comparación ******** //
            bool operator==(const Quaternion& q) const;
            bool operator!=(const Quaternion& q) const;
            
            // Función que devuelve una representación
            // del cuaternión como texto:
            std::string             ToString() const;
            
            // Función que devuelve el módulo del cuaternión:
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
                                                    double& angle);

            // Quaterniones unitarios colineales a las componentes w, x, y, z:
            static Quaternion   identity;
            static Quaternion   i;
            static Quaternion   j;
            static Quaternion   k;
            
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
