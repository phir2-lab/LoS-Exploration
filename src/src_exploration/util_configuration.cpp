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

#include <src_exploration/util_configurations.h>

UtilConfigurations::UtilConfigurations()
{

}

UtilConfigurations::UtilConfigurations(string adress)
{
    this->Load(adress);
}

bool UtilConfigurations::Load(string adress)
{
    ifstream file;
    file.open(adress);

    if(!file.is_open()){
        std::cout << std::endl << "ERROR: " << "Cannot read configuration file: " << adress << std::endl;
        return false;
    }

    string line;
    while(getline(file,line))
    {
        line = RemoveComment(line);
        istringstream is_line(line);
        string key;
        if(getline(is_line, key, '='))
        {
            string value;
            if(getline(is_line, value) )
            {
                key = RemoveSpace(key);
                value = RemoveSpace(value);
                values_[key] = value;
            }

        }
    }

    return true;
}

bool UtilConfigurations::GetString(string key, string& dst)
{
    map<string, string>::iterator it = values_.find(key);

    if (it != values_.end())
    {
        dst = it->second;
        return true;
    }

    return false;
}

bool UtilConfigurations::GetInt(string key, int& dst)
{
    map<string, string>::iterator it = values_.find(key);

    if (it != values_.end())
    {
        dst = stoi(it->second);
        return true;
    }

    return false;
}

bool UtilConfigurations::GetFloat(string key, float &dst)
{
    map<string, string>::iterator it = values_.find(key);

    if (it != values_.end())
    {
        dst = stof(it->second);
        return true;
    }

    return false;
}

bool UtilConfigurations::GetDouble(string key, double &dst)
{
    map<string, string>::iterator it = values_.find(key);

    if (it != values_.end())
    {
        dst = stod(it->second);
        return true;
    }

    return false;
}

bool UtilConfigurations::GetBool(string key, bool &dst)
{
    map<string, string>::iterator it = values_.find(key);

    if (it != values_.end())
    {
        if(it->second.compare("true") == 0 || it->second.compare("True") == 0 || it->second.compare("TRUE") == 0)
        {
            dst = true;
            return true;
        }
        else if(it->second.compare("false") == 0 || it->second.compare("False") == 0 || it->second.compare("FALSE") == 0)
        {
            dst = false;
            return true;
        }
        else{
            std::cout << std::endl << "ERROR: " << "Parameter " << key << " in configurations is not a correct bool expression. (" << it->second << ")" << std::endl;
            return false;
        }
    }

    return false;
}

string UtilConfigurations::GetString(string key)
{
    map<string, string>::iterator it = values_.find(key);

    if (it != values_.end())
        return it->second;

    std::cout << std::endl << "ERROR: " << "Cannot find parameter " << key << " in configurations." << std::endl;
    return "";
}

int UtilConfigurations::GetInt(string key)
{
    map<string, string>::iterator it = values_.find(key);

    if (it != values_.end())
        return stoi(it->second);

    std::cout << std::endl << "ERROR: " << "Cannot find parameter " << key << " in configurations." << std::endl;
    return -1;
}

float UtilConfigurations::GetFloat(string key)
{
    map<string, string>::iterator it = values_.find(key);

    if (it != values_.end())
    {
        istringstream buffer(it->second);
        float number;
        buffer >> number;
        return number;
    }

    std::cout << std::endl << "ERROR: " << "Cannot find parameter " << key << " in configurations." << std::endl;
    return -1;
}

double UtilConfigurations::GetDouble(string key)
{
    map<string, string>::iterator it = values_.find(key);

    if (it != values_.end())
    {
        istringstream buffer(it->second);
        double number;
        buffer >> number;
        return number;
    }

    std::cout << std::endl << "ERROR: " << "Cannot find parameter " << key << " in configurations." << std::endl;
    return -1;
}

bool UtilConfigurations::GetBool(string key)
{
    map<string, string>::iterator it = values_.find(key);

    if (it != values_.end())
    {
        if(it->second.compare("true") == 0 || it->second.compare("True") == 0 || it->second.compare("TRUE") == 0)
        {
            return true;
        }
        else if(it->second.compare("false") == 0 || it->second.compare("False") == 0 || it->second.compare("FALSE") == 0)
        {
            return false;
        }
        else{
            std::cout << std::endl << "ERROR: " << "Parameter " << key << " in configurations is not a correct bool expression.(" << it->second << ")" << std::endl;
            return false;
        }
    }

    std::cout << std::endl << "ERROR: " << "Cannot find parameter " << key << " in configurations." << std::endl;
    return false;
}

const string UtilConfigurations::RemoveSpace(string original)
{
    for (size_t i = 0; i < original.length(); i++)
    {
        if(original[i] == ' ' || original[i] == '\n' || original[i] == '\t') {
            original.erase(i, 1);
            i--;
        }
    }
    return original;
}

const string UtilConfigurations::RemoveComment(string original)
{
    string comment_marker = "#";
    size_t pos = 0;
    string token;

    while ((pos = original.find(comment_marker)) != string::npos) {
        token = original.substr(0, pos);
        original.erase(pos, string::npos);
    }
    return original;
}
