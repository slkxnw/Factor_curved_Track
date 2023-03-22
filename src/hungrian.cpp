# include "factor_curved_track/hungarian.h"

namespace my_match{
//https://www.cnblogs.com/techflow/p/13522712.html

bool hungrian::match()
{
    match_num = 0;
    //尝试为，每一位“工人”分配一个“工作”，如果分配成功，返回值是1，match_num++
    for(int i = 0; i < n; i++){
        used = std::vector<int> (n, 0);
        match_num += find_match(i);
    }
    //如果全部分配成功，就返回true
    if(match_num == n)
        return true;
    return false;
}

bool hungrian::find_match(int i)
{
    //在这个循环中，used会记录，已经递归到的工人的能够匹配的工作，也就是集合T，它也算是NS
    //在这一个完整的遍历中，实际上会进行NS==T的比较，但因为没有设置退出条件，提前满足NS==T的时候，不会退出，效率会有所下降
    //尝试，将每一个“工作”分给i这个“工人”
    for(int j = 0; j < n; j++)
    {
        //如果graph表示第j份“工作”和i这个“工人”有联系
        //并且第j份“工作”，没有尝试分配给“工人”
        if(graph[i][j] == 1 && used[j] == 0)
        {
            //尝试将这个工作分配给工人
            used[j] = 1;
            //如果第j份工作实际上没有分配到人
            //或者，第j份工作分配到的人，可以找到其他的工作（重新分配的这份工作不能分给工人i）
            if(matches[j] == -1 || find_match(matches[j]))
            {
                //分配成功
                matches[j] = i;
                return true;
            }
        }
    }
    return false;
}


}