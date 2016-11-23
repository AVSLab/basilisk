/*
Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder

Permission to use, copy, modify, and/or distribute this software for any
purpose with or without fee is hereby granted, provided that the above
copyright notice and this permission notice appear in all copies.

THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

*/

#include "coeffLoader.h"
#include <stdio.h>
#include <string>
// basic file operations
#include <iostream>
#include <fstream>
#include <ctype.h>


///-------------------------------coeffLoader--------------------------------///
coeffLoader::coeffLoader()
{
    this->_errorMessage = std::string("");
}

coeffLoader::coeffLoader(const coeffLoader& x)
{
    this->_errorMessage = x.getLastErrorMessage();
}

coeffLoader::~coeffLoader()
{
    
}

coeffLoader& coeffLoader::operator=(const coeffLoader& x)
{
    if (&x == this)
        return *this;
    
    this->_errorMessage = x.getLastErrorMessage();
    
    return *this;
}

/*!
 @brief Use this method to get the last error message.
 @return A string with the message.
 */
std::string coeffLoader::getLastErrorMessage(void) const
{
    return this->_errorMessage;
}

/*!
 @brief Transforms the exponent character from D to E. Some coefficient files use D instead of E.
 */
void coeffLoader::replaceExpDesignator(std::string& str)
{
    std::string::iterator cii;
    
    for (cii = str.begin(); cii < str.end(); cii++)
    {
        if (*cii == 'D')
            *cii = 'E';
    }
    
    return;
}

///----------------------------coeffLoaderTest-----------------------------///
coeffLoaderTest::coeffLoaderTest()
    : coeffLoader()
{
    
}

coeffLoaderTest::~coeffLoaderTest()
{
    
}

bool coeffLoaderTest::load(const std::string& filename, double** C_bar, double** S_bar, unsigned int* degree)
{
    *degree = 10;
    
    
    for (unsigned int l = 0; l <= *degree; l++)
    {
        for (unsigned int m = 0; m <= l; m++)
        {
            C_bar[l][m] = (2*(*degree) - l - m + 1)/(*degree);
            S_bar[l][m] = (2*(*degree) - l - m + 1)/(*degree);
        }
    }
    
    return true;    
}

///----------------------------coeffLoaderCSV------------------------------///
coeffLoaderCSV::coeffLoaderCSV() :
    coeffLoader()
{
    
}

coeffLoaderCSV::coeffLoaderCSV(const coeffLoaderCSV& x) :
    coeffLoader(x)
{
    
}

coeffLoaderCSV::~coeffLoaderCSV()
{
    
}

/*!
 @brief Loads the coefficients into the pre-allocated matrices C_bar and S_bar.
 @param[in] filename Name of the file (and route) to be processed.
 @param[out] C_bar Array where the C coefficients are to be loaded. The array must be pre-allocated.
 @param[out] S_bar Array where the S coefficients are to be loaded. The array must be pre-allocated.
 @param[in-out] max_degree It specifies the maximum degree to be loaded. If the maximum degree present in the file is smaller, max_degree is modified.
 */
bool coeffLoaderCSV::load(const std::string& filename, double** C_bar, double** S_bar, unsigned int* max_degree)
{
    std::ifstream f;
    std::string line;
    std::string::iterator cii;
    std::string::iterator initial;
    
    int param_nmber;
    unsigned int degree = 0;
    unsigned int order = 0;
    double C_lm = 0;
    double S_lm = 0;
    long index;
    std::string aux;
    std::string::size_type sz;     // alias of size_t
    bool degreeLocked = false;
    
    f.open(filename, std::ifstream::in);
    if (f.fail())
    {
        this->_errorMessage = "ERROR: The file could not be open.";
        return false;
    }
    
    while (getline(f, line))
    {
        if (degree > *max_degree)
            break;
        
        initial = line.begin();
        
        this->replaceExpDesignator(line); // Replace D with E if necessary
        
        param_nmber = 0;
        
        cii = initial;
        while (cii != line.end())
        {
//            if (*cii == this->_separationChar || *cii == this->_separationChar)
            if (!isdigit(*cii) && *cii != '-')
            {
                cii++;
                continue;
            }
            
            if (param_nmber == 0)       // Degree
            {
                double entry;
                
                index = distance(initial, cii);
                aux = line.substr(index, std::string::npos);
                entry = stod(aux, &sz);
                degree = (unsigned int) entry;
                
                if (entry - degree != 0) { // Line doesn't have the degree first
                    degree = 0;
                    break;
                }
                
                if (degree > *max_degree)
                {
                    degree = degreeLocked ? degree : 0;
                    break;
                }

                degreeLocked = true;
                cii += sz;
                param_nmber++;
            }
            else if (param_nmber == 1)  // Order
            {
                index = distance(initial, cii);
                aux = line.substr(index, std::string::npos);
                order = stod(aux, &sz);
                cii += sz;
                param_nmber++;
            }
            else if (param_nmber == 2)  //C_bar
            {
                index = distance(initial, cii);
                aux = line.substr(index, std::string::npos);
                C_lm = stod(aux, &sz);
                cii += sz;
                param_nmber++;
            }
            else if (param_nmber == 3)  //S_bar
            {
                index = distance(initial, cii);
                aux = line.substr(index, std::string::npos);
                S_lm = stod(aux, &sz);
                cii += sz;
                param_nmber++;
            }
            else
            {
                C_bar[degree][order] = C_lm;
                S_bar[degree][order] = S_lm;
                
                break;
            }
        }
    }
    
    if (*max_degree > degree)
        *max_degree = degree;
    
    f.close();
    
    return true;
}