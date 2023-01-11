#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Core>

#include "mybackend.h"
using namespace std;


int main(int argc, char **argv)
{
    if (argc != 2)
    {
        cout << "no filenale"<<endl;
        return 1;
    }
    ifstream fin(argv[1]);
    if (!fin)
    {
        cout<<"file"<<argv[1]<< "does not exist"<<endl;
        return 1;
    }

    
}