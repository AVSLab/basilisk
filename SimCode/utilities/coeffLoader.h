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
