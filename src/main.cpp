#include "maindeal.h"

int main(int argc, char *argv[])
{
    Maindeal m(argc,argv);
    return m.getApp()->exec();
}

