#include <iostream>
#include <string>

int main() {

    string input_str;
    getline(cin,input_str);//只有0、1
    
    string temp,last_temp; 
    vector<string> output_group;

    for(int i=0;i<input_str.size();i++){
        string temp_first=input_str[i];
        last_temp=temp_first;
        string output_str=temp_first;
        for(int j=i+1;j<input_str.size();j++)
        {
            temp=input_str[j];
            if(temp==last_temp){
                continue;
            }
            else{
                output_str=output_str+temp;
                last_temp=temp;
            }
            if(output_str.size()==3)//找到了
            {
                output_group.push_back(output_str);
                output_str=temp_first;
            }
        }
    
    }
    std::cout<<output_group.size()<<std::endl;

    return 0;
}