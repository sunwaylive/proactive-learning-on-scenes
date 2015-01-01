#pragma once
#include <omp.h>
#pragma warning(disable: 4786)
#include <iomanip>
#include <vector>
#include <map>
#include <stack>
#include <set>
#include <list>
#include <queue>
#include <limits>
#include <cassert>
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#define _USE_MATH_DEFINES
#include <math.h>
const double LENGTH_EPSILON_CONTROL = 1e-6;
const double RateOfNormalShift = 5e-3;
const double ToleranceOfConvexAngle = 5e-2;

using namespace std;