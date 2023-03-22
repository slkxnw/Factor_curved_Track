#ifndef MYTRK_HUNGRY
#define MYTRK_HUNGRY


# include"mycommon_include.h"

namespace my_match{
//https://www.cnblogs.com/techflow/p/13522712.html
class hungrian
{
    hungrian();
    bool match();

    void creat_zero_simple()
    {
        for(int i = 0; i < n; i++)
        {
            int min = *std::min_element(graph_iou[i].begin(), graph_iou[i].end());
            std::for_each(graph_iou[i].begin(), graph_iou[i].end(),
                          [&min](int & val){val -= min;});
        }
        for(int i = 0; i < n; i++){
            std::vector<int> tmp;
            for(int j = 0; j < n; j++)
                tmp.push_back(graph_iou[j][i]);
            int min = *std::min_element(tmp.begin(), tmp.end());
            for(int j = 0; j < n; j++)
                graph_iou[j][i] -= min;
        }
        

    }

private:

    bool find_match(int i);
    //dets和trks的数量
    int n;
    //表征“工人”和“工作”是否可以建立联系，是一个01矩阵
    std::vector<std::vector<int>> graph;
    //原始的“工人”和“工作”效用数据
    std::vector<std::vector<int>> graph_iou;
    //记录“工人”对应的“工作”的id
    std::vector<int> matches;
    //记录“工作”是否被分配了
    std::vector<int> used;
    int match_num;


};

}

#endif