/**
 * This file is part of LoS-Exploration.
 *
 * Copyright 2020 Diego Pittol <dpittol at inf dot ufrgs dot br> (Phi Robotics Research Lab - UFRGS)
 * For more information see <https://github.com/phir2-lab/LoS-Exploration>
 *
 * LoS-Exploration is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * LoS-Exploration is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with LoS-Exploration. If not, see <https://www.gnu.org/licenses/>.
**/

#ifndef UTILCONFIGURATIONS_H
#define UTILCONFIGURATIONS_H

#include <iostream>
#include <map>
#include <fstream>
#include <sstream>
#include <string>

using namespace std;

class UtilConfigurations
{
public:
    UtilConfigurations();
    UtilConfigurations(string adress);

    const string RemoveComment(string original);
    const string RemoveSpace(string original);

    bool Load(string adress);

    bool GetString(string key, string& dst);
    bool GetInt(string key, int &dst);
    bool GetFloat(string key, float &dst);
    bool GetDouble(string key, double &dst);
    bool GetBool(string key, bool &dst);

    string GetString(string key);
    int GetInt(string key);
    float GetFloat(string key);
    double GetDouble(string key);
    bool GetBool(string key);

private:
    map<string,string> values_;

};

#endif // UTILCONFIGURATIONS_H
