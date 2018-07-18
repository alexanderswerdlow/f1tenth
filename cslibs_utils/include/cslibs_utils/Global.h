/*
 * @file Global.h
 *
 * @date Jul 26, 2009 early 21st century
 * @author bohlmann
 */

#ifndef GLOBAL_H_
#define GLOBAL_H_
#include <vector>
#include <list>
#include <string>
#include <Eigen/Core>
#include <iostream>

// some types with defined length
typedef unsigned char       U8;
typedef unsigned short int  U16;
typedef unsigned int        U32;
typedef unsigned long long  U64;
typedef char                S8;
typedef short int           S16;
typedef int                 S32;
typedef long long           S64;
typedef unsigned int        Uint;

enum {
    EOK                     =   0, //  status ok
    EINTERNAL               =  -1, // general internal error
    ENOTFOUND               =  -2, // general not-found-error
    ESYSTEM                 =  -3, // general error resulting from failed syscall
    ENOINIT                 =  -4, // general -something-not-initialised-error
    EFILEIO                 =  -5, // general file read/write/access error
    EINVALID                =  -6 // general invalid argument
};

typedef std::vector<int> IVector;
typedef std::vector<double> DVector;
typedef std::vector<float> FVector;
typedef std::list<int> IList;
typedef std::list<double> DList;
typedef std::list<float> FList;
typedef std::list<std::string> StringList;
typedef std::vector<std::string> StringVector;
typedef std::list<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > Vector2dList;
typedef std::list<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > Vector3dList;
typedef std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > Vector2dVec;
typedef std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > Vector3dVec;

#define MSGOUT(message) {\
    char filepath[] = __FILE__;\
    char* period = strrchr(filepath, '.');\
    char* slash = strrchr(filepath, '/');\
    char filename[32] = "";\
    strncpy(filename, slash + 1, period - slash - 1);\
    std::cout << filename << ": " << message << std::endl;\
}

#define VOUT(message) if (mVerbose) MSGOUT(__FUNCTION__ << "(): " << message);

#define EOUT(message) {\
    char filepath[] = __FILE__;\
    char* period = strrchr(filepath, '.');\
    char* slash = strrchr(filepath, '/');\
    char filename[32] = "";\
    strncpy(filename, slash + 1, period - slash - 1);\
    std::cerr << filename << ": ERROR: " << message << std::endl;\
}

#endif /* GLOBAL_H_ */
