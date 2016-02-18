//
//  coeffLoader.h
//  SphericalHarmonics
//
//  Created by Manuel Diaz Ramos on 12/19/15.
//  Copyright © 2015 Manuel Diaz Ramos. All rights reserved.
//

#ifndef coeffLoader_h
#define coeffLoader_h

#include <string>

/*!
 @brief Abstract class that must be inherited to process files with spherical harmonics coefficients.
 */
class coeffLoader
{
public:
    coeffLoader();
    coeffLoader(const coeffLoader& x);
    virtual ~coeffLoader();
    virtual bool load(const std::string& filename, double** C_bar, double** S_bar, unsigned int* max_degree) = 0;
    
    //Overloaded operators
    coeffLoader& operator=(const coeffLoader& x);
    
    std::string getLastErrorMessage(void) const;
    
protected:
    std::string _errorMessage;
    void replaceExpDesignator(std::string& str);
};

/*!
 @brief Generates a test set of coefficients.
 */
class coeffLoaderTest : public coeffLoader
{
public:
    coeffLoaderTest();
    virtual ~coeffLoaderTest();
    virtual bool load(const std::string& filename, double** C_bar, double** S_bar, unsigned int* max_degree);
};

/*!
 @brief Loads a set of coefficients taken from a CSV file.
 */
class coeffLoaderCSV : public coeffLoader
{
public:
    coeffLoaderCSV();
    coeffLoaderCSV(const coeffLoaderCSV& x);
    virtual ~coeffLoaderCSV();
    virtual bool load(const std::string& filename, double** C_bar, double** S_bar, unsigned int* max_degree);

private:
    //unsigned char _separationChar;
};

#endif /* coeffLoader_hpp */
