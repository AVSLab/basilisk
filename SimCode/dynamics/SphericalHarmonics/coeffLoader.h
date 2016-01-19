//
//  coeffLoader.hpp
//  SphericalHarmonics
//
//  Created by Manuel Diaz Ramos on 12/19/15.
//  Copyright © 2015 Manuel Diaz Ramos. All rights reserved.
//

#ifndef coeffLoader_h
#define coeffLoader_h

#include <string>

using namespace std;

/*!
 @brief Abstract class that must be inherited to process files with spherical harmonics coefficients.
 */
class coeffLoader
{
public:
    coeffLoader();
    virtual ~coeffLoader();
    virtual bool load(const string& filename, double** C_bar, double** S_bar, unsigned int* max_degree) = 0;
    
    string getLastErrorMessage(void);
    
protected:
    string _errorMessage;
    void replaceExpDesignator(string& str);
};

/*!
 @brief Generates a test set of coefficients.
 */
class coeffLoaderTest : public coeffLoader
{
public:
    coeffLoaderTest();
    virtual ~coeffLoaderTest();
    virtual bool load(const string& filename, double** C_bar, double** S_bar, unsigned int* max_degree);
};

/*!
 @brief Loads a set of coefficients taken from a CSV file.
 */
class coeffLoaderCSV : public coeffLoader
{
public:
    coeffLoaderCSV(const unsigned char separation_char);
    virtual ~coeffLoaderCSV();
    virtual bool load(const string& filename, double** C_bar, double** S_bar, unsigned int* max_degree);
    
private:
    unsigned char _separationChar;
};

#endif /* coeffLoader_hpp */
