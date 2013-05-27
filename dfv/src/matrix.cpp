#include <dfv/matrix.h>

namespace dfv
{

    Matrix::Matrix(): rows(0), columns(0)
    {

    }

    Matrix::Matrix(unsigned int size)
    {
        this->Create(size, size);
    }

    Matrix::Matrix(unsigned int rows, unsigned int columns)
    {
        this->Create(rows, columns);
    }

    Matrix::~Matrix()
    {
        //dtor
    }
    
    Matrix& Matrix::operator=(const Matrix& m)
    {
        if(this != &m)
        {
            this->Create(m.GetRows(), m.GetColumns());
            
            for(unsigned int j = 0; j < this->GetRows(); j++)
            {
                for(unsigned int i = 0; i < this->GetColumns(); i++)
                {
                    this->Set(j, i, m.Get(j, i));
                }
            }
        }
        
        return *this;
    }
    
    Matrix& Matrix::operator+=(const Matrix& m)
    {
        if(m.GetRows() == this->GetRows() && m.GetColumns() == this->GetColumns())
        {        
            for(unsigned int j = 0; j < this->GetRows(); j++)
            {
                for(unsigned int i = 0; i < this->GetColumns(); i++)
                {
                    this->Set(j, i, this->Get(j, i) + m.Get(j, i));
                }
            }
            
            return *this;
        }
        else
        {
            return *this; // arrojar error 
        }
    }
    
    Matrix& Matrix::operator-=(const Matrix& m)
    {
        if(m.GetRows() == this->GetRows() && m.GetColumns() == this->GetColumns())
        {        
            for(unsigned int j = 0; j < this->GetRows(); j++)
            {
                for(unsigned int i = 0; i < this->GetColumns(); i++)
                {
                    this->Set(j, i, this->Get(j, i) - m.Get(j, i));
                }
            }
            
            return *this;
        }
        else
        {
            return *this; // arrojar error 
        }
    }
    
    Matrix& Matrix::operator*=(const double k)
    {       
        for(unsigned int j = 0; j < this->GetRows(); j++)
        {
            for(unsigned int i = 0; i < this->GetColumns(); i++)
            {
                this->Set(j, i, this->Get(j, i) * k);
            }
        }
        
        return *this;
    }
    
    const Matrix Matrix::operator+(const Matrix& m) const
    {
        return Matrix(*this) += m;
    }
    
    const Matrix Matrix::operator-(const Matrix& m) const
    {
        return Matrix(*this) -= m;
    }
    
    const Matrix operator*(double k, Matrix& m)
    {
        return Matrix(m) *= k;
    }
    
    const Matrix Matrix::operator*(double k) const
    {
        return Matrix(*this) *= k;
    }
    
    const Matrix Matrix::operator*(const Matrix& m) const
    {
        Matrix result;
        if(this->GetColumns() == m.GetRows())
        {
            result.Create(this->GetRows(), m.GetColumns());
            for(unsigned int j = 0; j < result.GetRows(); j++)
            {
                for(unsigned int i = 0; i < result.GetColumns(); i++)
                {
                    for(unsigned int k = 0; k < this->GetColumns(); k++)
                    {
                        result.Set(j, i, result.Get(j, i) + this->Get(j, k)*m.Get(k, i));
                    }
                }
            }
        }
        return result;
    }
    
    bool Matrix::operator==(const Matrix& m) const
    {
        if(m.GetRows() == this->GetRows() && m.GetColumns() == this->GetColumns())
        {        
            for(unsigned int j = 0; j < this->GetRows(); j++)
            {
                for(unsigned int i = 0; i < this->GetColumns(); i++)
                {
                    if(this->Get(j, i) != m.Get(j, i))
                    {
                        return false;
                    }
                }
            }
            
            return true;
        }
        else
        {
            return false;
        }
    }
    
    bool Matrix::operator!=(const Matrix& m) const
    {
        return !(*this == m);
    }

    std::string Matrix::ToString() const
    {
        std::stringstream ss;
        
        for(unsigned int j = 0; j < this->GetRows(); j++)
        {
            for(unsigned int i = 0; i < this->GetColumns(); i++)
            {
                if(this->Get(j, i) >= 0)
                {
                    ss << " ";
                }
                ss << this->Get(j, i) << "\t";
            }
            ss << std::endl;
        }
        
        return ss.str();
    }

    Matrix& Matrix::Create(unsigned int rows, unsigned int columns, double value)
    {
        this->rows = rows;
        this->columns = columns;
        this->data.resize(rows*columns);
        typename std::vector<double>::iterator it;
        for(it = this->data.begin(); it != this->data.end(); it++)
        {
            *it = value;
        }
        return *this;
    }

    double Matrix::Get(unsigned int row, unsigned int col) const
    {
        return this->data.at(row + col*this->columns);
    }

    void Matrix::Set(unsigned int row, unsigned int col, double value)
    {
        this->data.at(row + col*this->columns) = value;
    }

    unsigned int Matrix::GetRows() const
    {
        return this->rows;
    }

    unsigned int Matrix::GetColumns() const
    {
        return this->columns;
    }

    Matrix Matrix::GetMinor(unsigned int row, unsigned int column) const
    {
        Matrix result;
        if(this->GetRows() == this->GetColumns())
        {
            result.Create(this->GetRows() - 1, this->GetColumns() - 1);
            unsigned int row_count = 0;
            unsigned int col_count = 0;
            for(unsigned int j = 0; j < this->GetRows(); j++)
            {
                if(j != row)
                {
                    col_count = 0;
                    for(unsigned int i = 0; i < this->GetColumns(); i++)
                    {
                        if(i != column)
                        {
                            result.Set(row_count, col_count, this->Get(j, i));
                            col_count++;
                        }
                    }
                    row_count++;
                }
            }
        }
        return result;
    }

    void Matrix::Randomize()
    {
        for(unsigned int j = 0; j < this->GetRows(); j++)
        {
            for(unsigned int i = 0; i < this->GetColumns(); i++)
            {
                this->Set(j, i, (double)rand() / (double)RAND_MAX);
            }
        }
    }

    double Matrix::GetDeterminant() const
    {
        double result = 0;
        if(this->GetRows() == this->GetColumns())
        {
            if(this->GetRows() == 1)
            {
                return this->Get(0, 0);
            }
            else
            {
                for(unsigned int i = 0; i < this->GetColumns(); i++)
                {
                    Matrix minor = this->GetMinor(0, i);
                    result += (i % 2 == 0? 1.0 : -1.0)*this->Get(0, i)*minor.GetDeterminant();
                }
            }
        }
        return result;
    }

    const Matrix Matrix::GetTransposed() const
    {
        Matrix result;
        result.Create(this->GetColumns(), this->GetRows());
        for(unsigned int j = 0; j < this->GetRows(); j++)
        {
            for(unsigned int i = 0; i < this->GetColumns(); i++)
            {
                result.Set(j, i, this->Get(i, j));
            }
        }
        return result;
    }

    const Matrix Matrix::GetAdjoint() const
    {
        Matrix result;
        result.Create(this->GetRows(), this->GetColumns());
        for(unsigned int j = 0; j < this->GetRows(); j++)
        {
            for(unsigned int i = 0; i < this->GetColumns(); i++)
            {
                Matrix minor;
                minor = this->GetMinor(j, i);
                double temp = ((j+i)%2 == 0 ? 1.0 : -1.0) * minor.GetDeterminant();
                result.Set(j, i, temp);
            }
        }
        return result;
    }

    const Matrix Matrix::GetAdjugate() const
    {
        Matrix result;
        result.Create(this->GetRows(), this->GetColumns());
        for(unsigned int j = 0; j < this->GetRows(); j++)
        {
            for(unsigned int i = 0; i < this->GetColumns(); i++)
            {
                Matrix minor;
                minor = this->GetMinor(j, i);
                double temp = ((j+i)%2 == 0 ? 1.0 : -1.0) * minor.GetDeterminant();
                result.Set(i, j, temp);
            }
        }
        return result;
    }

    const Matrix Matrix::GetInverse() const
    {
        Matrix result;
        if((this->GetRows() == this->GetColumns()) && this->GetDeterminant() != 0.0)
        {
            Matrix temp = this->GetAdjugate();
            temp *= (1.0 / this->GetDeterminant());
            result = temp;
        }
        return result;
    }

    double Matrix::operator()(unsigned int row, unsigned int column) const
    {
        return this->Get(row, column);
    }
    
    const Matrix Matrix::Identity(unsigned int size)
    {
        Matrix result(size, size);
        
        for(int i = 0; i < size; ++i)
        {
            result.Set(i, i, 1.0);
        }
        
        return result;
    }

    std::ostream& operator<<(std::ostream& os, Matrix& m)
    {
        os << m.ToString();
        return os;
    }

}
