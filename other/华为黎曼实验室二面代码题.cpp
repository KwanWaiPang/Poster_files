


bool IsRightStr(std::string str)
{
    // 堆栈
    stack<char> stack_group;

    for(int i=0;i<str.size();i++)
    {
        if(str[i]=='('|| str[i]=='['|| str[i]=='{' )//左括号
        {
            stack_group.push(str[i]);//入栈
        }
        else if(str[i]==')'|| str[i]==']'|| str[i]=='}' )
        {
            if(stack_group.empty())//若为空的就false
            {
                return false;
            }
            else{
                if (stack_group.top() ==str[i])
                    stack_group.pop();//删掉
                else
                    return false;//返回报错
            }
        }
    }

    // 最后再次检查堆是否为空
    if(stack_group.empty())
        return false;

    return true;

}