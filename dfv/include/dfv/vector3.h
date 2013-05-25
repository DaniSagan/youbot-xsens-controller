/* Clase Vector3
 * Incluye operaciones para creación y manipulación
 * de vectores de 3 dimensiones
 *
 * Autor: Daniel Fernández Villanueva
 */
 
#ifndef VECTOR3_H
#define VECTOR3_H

#include <string>
#include <sstream>
#include <cmath>
#include <dfv/quaternion.h>

namespace dfv
{

    class Quaternion;

    class Vector3
    {
        public:
            Vector3();
            Vector3(double x_, double y_, double z_);
            explicit Vector3(const Quaternion& q);
            virtual ~Vector3();
            
            // ******** Operador de asignación ******** //
            Vector3& operator=(const Vector3& v);
            
            // ******** Operadores de asignación compuestos ******** //
            Vector3& operator+=(const Vector3& v);
            Vector3& operator-=(const Vector3& v);
            Vector3& operator*=(const double k);
            
            // ******** Operadores aritméticos binarios ******** //
            const Vector3 operator+(const Vector3& v) const;
            const Vector3 operator-(const Vector3& v) const;            
            friend const Vector3 operator*(double k, Vector3& v);
            const Vector3 operator*(double k) const;
            // Producto escalar:
            double operator*(const Vector3& v) const;            
            // Producto vectorial:
            const Vector3 operator^(const Vector3& v) const;
            
            // ******** Operadores de comparación ******** //
            bool operator==(const Vector3& v) const;
            bool operator!=(const Vector3& v) const;
            
            // Funcion que devuelve la magnitud del vector:
            double          GetMagnitude() const;
            
            // Funcion que normaliza el vector:
            void            Normalize();
            
            // Funcion que devuelve el vector normalizado:
            const Vector3   GetNormalized() const;
            
            // Funcion que devuelve un vector de longitud
            // k veces el original con la misma dirección
            // y sentido:
            const Vector3   GetScalated(double k) const;
            
            // Función que rota el vector de acuerdo con el cuaternión
            // pasado como parámetro:
            Vector3&        Rotate(const Quaternion q);
            
            // Devuelve el vector rotado según el cuaternión pasado
            // como parámetro:
            const Vector3   GetRotated(const Quaternion& q) const;
            
            // Función que devuelve una representación
            // del vector como texto:
            std::string     ToString() const;
            
            // Vector unitario codireccional al eje x:
            static Vector3 i;
            
            // Vector unitario codireccional al eje y:
            static Vector3 j;
            
            // Vector unitario codireccional al eje z:
            static Vector3 k;
            
            // Componentes del vector:
            double x;
            double y;
            double z;
            
        protected:
        private:
            
    };
    
    std::ostream& operator<<(std::ostream& os, const Vector3& v);
    
}

#endif // Vector3_H
