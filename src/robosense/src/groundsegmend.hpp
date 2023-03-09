/*日期：2020年2月13日
作者：szh
功能：地面分割算法
须知：单位是m*/

using namespace std;
#define MASK_BG_COLOR 128
#define MASK_FG_COLOR 250

class gSegmend
{
private:
    double gridySeg[105];     //环状栅格距离设置
    double lineDistance[128]; //线间距离
public:
    gridFactory mgridfac;
    double smooth_value; 
    int pra_ground;
   
    //栅格图纵向距离与内存初始化
    gSegmend(){
        smooth_value=2;
        pra_ground=1;

        gridySeg[0] = 0;
        gridySeg[1] = 5;
        for(size_t num=2;num<104;num++){
            gridySeg[num] = gridySeg[num-1]+pra_ground;  //1米一段珊格
        }
        gridySeg[104]=1000;
        mgridfac.initGrid(320, 105);

        //反向搜索的线间距
        lineDistance[0] = 1;  //1-2线的距离
        lineDistance[1] = 1;
        lineDistance[2] = 0.8;
        lineDistance[3] = 0.8;
        lineDistance[4] = 0.8;
        lineDistance[5] = 0.6;
        lineDistance[6] = 0.5;
        lineDistance[7] = 0.3;
        for(int i=8;i<15;i++){
            lineDistance[i] = 0.08;
        }
        for(int i=15;i<31;i++){
            lineDistance[i]=0.05;
        }
        for(int i=31;i<47;i++){
            lineDistance[i] = 0.1;
        }
        for(int i=47;i<63;i++){
            lineDistance[i] = 0.3;
        }
        for(int i=63;i<79;i++){
            lineDistance[i] = 0.3;
        }
        for(int i=79;i<97;i++){
            lineDistance[i] = 0.5;
        }
        for(int i=97;i<115;i++){
            lineDistance[i] = 0.8;
        }
        lineDistance[115]=1;
        lineDistance[116]=1;
        lineDistance[117]=1;
        lineDistance[118]=1;
        lineDistance[119]=1;
        lineDistance[120]=1;
        lineDistance[121]=1.5;
        lineDistance[122]=1.5;
        lineDistance[123]=1.5;
        lineDistance[124]=1.5;
        lineDistance[125]=2;
        lineDistance[126]=2;
        lineDistance[127]=2;
         for(int i=0;i<LINE;i++){
                lineDistance[i]*=smooth_value;
            }

    }

    void setparamground(){
        double tmp_smooth =2;
        //p->param<double>("smooth",tmp_smooth,2);
        if(tmp_smooth != smooth_value){
            smooth_value = tmp_smooth;
            lineDistance[0] = 1;  //1-2线的距离
            lineDistance[1] = 1;
            lineDistance[2] = 0.8;
            lineDistance[3] = 0.8;
            lineDistance[4] = 0.8;
            lineDistance[5] = 0.6;
            lineDistance[6] = 0.5;
            lineDistance[7] = 0.3;
            for(int i=8;i<15;i++){
                lineDistance[i] = 0.08;
            }
            for(int i=15;i<31;i++){
                lineDistance[i]=0.05;
            }
            for(int i=31;i<47;i++){
                lineDistance[i] = 0.1;
            }
            for(int i=47;i<63;i++){
                lineDistance[i] = 0.3;
            }
            for(int i=63;i<79;i++){
                lineDistance[i] = 0.3;
            }
            for(int i=79;i<97;i++){
                lineDistance[i] = 0.5;
            }
            for(int i=97;i<115;i++){
                lineDistance[i] = 0.8;
            }
            lineDistance[115]=1;
            lineDistance[116]=1;
            lineDistance[117]=1;
            lineDistance[118]=1;
            lineDistance[119]=1;
            lineDistance[120]=1;
            lineDistance[121]=1.5;
            lineDistance[122]=1.5;
            lineDistance[123]=1.5;
            lineDistance[124]=1.5;
            lineDistance[125]=2;
            lineDistance[126]=2;
            lineDistance[127]=2;
            for(int i=0;i<LINE;i++){
                lineDistance[i]*=smooth_value;
            }
        }
        printf("smooth value is %f\n",smooth_value);

        
        double tmp_ground = 1;
        //p->param<double>("groundprarm",tmp_ground,1);
        if(tmp_ground != pra_ground)
        {
            pra_ground = tmp_ground;
            gridySeg[0] = 0;
            gridySeg[1] = 5;
            for(size_t num=2;num<104;num++){
                gridySeg[num] = gridySeg[num-1]+pra_ground;  //1米一段珊格
            }
            gridySeg[104]=1000;
            mgridfac.initGrid(320, 105);
        }
        printf("ground value is %d\n",pra_ground);
        
    }

    /// 栅格落点计算
    /// \param ptDistance
    /// \return
    int getgridy(double ptDistance)
    {
        int result =0;
        if(ptDistance<gridySeg[1]){
            return result;
        }
        else if(ptDistance>=gridySeg[1] && ptDistance<gridySeg[103]){
            result = (ptDistance - gridySeg[1])/pra_ground+1;
            return result;
        }
        else{
            return 103;
        }
    }

    /// 环状栅格高度差
    /// \param mcloud
    void circleGroundSeg(pointCloud *mcloud)
    {

        mgridfac.clearGrid();

        //栅格高度统计
        for (size_t c = 0; c < mcloud->circlelen; c++)
        {
            int gridx = c / 6;
            for (size_t l = 0; l < LINE; l++)
            {
                if(mcloud->mptclout[l][c].type==0){
                    continue;
                }   
                float x = mcloud->mptclout[l][c].x;
                float y = mcloud->mptclout[l][c].y;
                float z = mcloud->mptclout[l][c].z;
                float d = sqrt(x * x + y * y);
                int gridy = getgridy(d);
                mcloud->mptclout[l][c].gridx = gridx;
                mcloud->mptclout[l][c].gridy = gridy;

                if (mgridfac.mgrid[gridy][gridx].high < z)
                {
                    mgridfac.mgrid[gridy][gridx].high = z;
                }
                if (mgridfac.mgrid[gridy][gridx].low > z)
                {
                    mgridfac.mgrid[gridy][gridx].low = z;
                }

                mgridfac.mgrid[gridy][gridx].density++; //栅格中点个数
            }
        }
        gridconnect();
        //依据栅格高度提取地面点
        for (size_t l = 0; l < LINE; l++)
            for (size_t c = 0; c < mcloud->circlelen; c++)
            {
                if(mcloud->mptclout[l][c].type==0){
                    continue;
                }  
               // 忽略一定高度的点
                
                int gridx = mcloud->mptclout[l][c].gridx;
                int gridy = mcloud->mptclout[l][c].gridy;
                double z = mcloud->mptclout[l][c].z;
                double min_z = mgridfac.mgrid[gridy][gridx].low;
                mcloud->mptclout[l][c].lowest = min_z;

                int density = 5;
                float altInCept = 0.2;

                ///颠簸路段用 分段高度差阈值
                if (mcloud->mptclout[l][c].d >= 30 && mcloud->mptclout[l][c].d < 50)
                {
                    density = 4;
                    altInCept = 0.25;
                }
                else if (mcloud->mptclout[l][c].d >= 50 && mcloud->mptclout[l][c].d < 80)
                {
                    density = 3;
                    altInCept = 0.3;
                }
                else if (mcloud->mptclout[l][c].d >= 80)
                {
                    density = 2;
                    altInCept = 0.5;
                }

                float delta_z = z - min_z;
                if (delta_z > altInCept && mgridfac.mgrid[gridy][gridx].density >= density)
                {
                    mcloud->mptclout[l][c].type = 20;
                    
                }

                else
                {
                    mcloud->mptclout[l][c].type = 10;
                    SpaceTimeGroundSegv2(mcloud,l,c); //根据反向搜索障碍物
                }
            }
    }

    /// 珊格联系
    void gridconnect(){
        //规定第一条线的高度范围小于0.1m且大于-0.1m
        for(int j=0;j<320;j++){
            if(mgridfac.mgrid[0][j].low>0.1){
                mgridfac.mgrid[0][j].low =0.1;
            }
            if(mgridfac.mgrid[0][j].low<-0.2){
                mgridfac.mgrid[0][j].low = -0.2;
            }

        }
        //规定后一条线的高度范围在前一条线的正负0.2m
        for(int j=0;j<320;j++){
            for(int i=1;i<105;i++){
                if(mgridfac.mgrid[i][j].low > mgridfac.mgrid[i-1][j].low+0.2){
                    mgridfac.mgrid[i][j].low = mgridfac.mgrid[i-1][j].low+0.2;
                }
                //针对水面反射点
                if(mgridfac.mgrid[i][j].low < mgridfac.mgrid[i-1][j].low-4){
                    mgridfac.mgrid[i][j].low = mgridfac.mgrid[i-1][j].low-4;
                }
            }
        }
    }

    /// 单线聚类，下降沿
    /// \param mcloud
    void Circleclustering(pointCloud *mcloud) {
        //增加1-10线的处理
        //水平聚类
        vector<vector<pt>> class_64;
        int C = mcloud->circlelen;
        for (int l = 0; l < 80; l++) {
            vector<pt> circle;
            pt tmp;
            //找到第一个有效点
            int num = 0;
            for (int c = 0; c < C; c++) {
                if (mcloud->mptclout[l][c].type == 10) {
                    num = c;
                    break;
                }
            }

            tmp.start = num;
            tmp.end = tmp.start;
            while (tmp.end < C) {
                //从有效地面点开始
                if (mcloud->mptclout[l][tmp.start].type == 0) {
                    tmp.start = tmp.start + 1;
                    tmp.end = tmp.start;
                    continue;
                }
                for (int c = tmp.end + 1; c < C; c++) {
                    // //无效点，立即断开
                    // if(mcloud->mptclout[l][c].type ==0){
                    //     break;
                    // }
                    //else{
                    if (ptxydistance(mcloud->mptclout[l][c].x,
                                     mcloud->mptclout[l][tmp.end].x,
                                     mcloud->mptclout[l][c].y,
                                     mcloud->mptclout[l][tmp.end].y) < 1) {
                        tmp.end = c;
                    } else {
                        break;
                    }
                    //}
                }
                //这一类少于3个点
                if (tmp.end - tmp.start <= 3) {
                    tmp.start = tmp.end + 1;
                    tmp.end = tmp.start;
                } else {
                    circle.push_back(tmp);
                    tmp.start = tmp.end + 1;
                    tmp.end = tmp.start;
                }
            }
            class_64.push_back(circle);

        }

        //打上标签
        for (int i = 0; i < class_64.size(); i++) {
            int label = 0;
            for (int j = 0; j < class_64[i].size(); j++) {
                int p1 = class_64[i][j].start;
                int p2 = class_64[i][j].end;
                for (int k = p1; k <= p2; k++) {
                    if (mcloud->mptclout[0 + i][k].type != 0)
                        mcloud->mptclout[0 + i][k].label = label;
                    //cout << "type="<<mcloud->mptclout[0+i][k].type<<endl;
                }
                label++;
            }
        }

        //看相邻类别的距离
        for (int i = 0; i < class_64.size(); i++) {
            if (class_64[i].size() <= 2) {
                continue;
            }
            for (int j = 1; j < class_64[i].size() - 1; j++) {
                int p1 = class_64[i][j].start;
                int p2 = class_64[i][j].end;

                //平均距离.又误检
                // float mean_d=0;
                // int count=1;
                // for(int k=p1;k<=p2;k++){
                //     if(mcloud->mptclout[0+i][k].type !=0){
                //         mean_d += dis(mcloud->mptclout[0+i][k].x,mcloud->mptclout[0+i][k].y);
                //         count++;
                //     }

                // }
                // mean_d /= count;

                //开头和结尾。误检少
                int ps, pe;
                for (int k = p1; k <= p2; k++) {
                    if (mcloud->mptclout[0 + i][k].type != 0) {
                        ps = k;
                        break;
                    }
                }
                for (int k = p2; k >= p1; k--) {
                    if (mcloud->mptclout[0 + i][k].type != 0) {
                        pe = k;
                        break;
                    }
                }
                //float ds=dis(mcloud->mptclout[0+i][ps].x,mcloud->mptclout[0+i][ps].y);
                //float de=dis(mcloud->mptclout[0+i][pe].x,mcloud->mptclout[0+i][pe].y);


                int pb;
                int pn;
                for (int b = class_64[i][j - 1].end; b >= class_64[i][j - 1].start; b--) {
                    if (mcloud->mptclout[0 + i][b].type != 0) {
                        pb = b;
                        break;
                    }
                }
                for (int e = class_64[i][j + 1].start; e <= class_64[i][j + 1].end; e++) {
                    if (mcloud->mptclout[0 + i][e].type != 0) {
                        pn = e;
                        break;
                    }
                }

                //float db=dis(mcloud->mptclout[0+i][pb].x,mcloud->mptclout[0+i][pb].y);
                //float dn=dis(mcloud->mptclout[0+i][pn].x,mcloud->mptclout[0+i][pn].y);

                float z1 = mcloud->mptclout[0 + i][ps].z;
                float z2 = mcloud->mptclout[0 + i][pe].z;
                float z3 = mcloud->mptclout[0 + i][pb].z;
                float z4 = mcloud->mptclout[0 + i][pn].z;
                //距离小于1米就是下降沿
                //if(ds<(db-1) && de<(dn-1)){
                if (fabs(z1 - z3) > 0.5) {
                    cout <<"l="<<i<<"pb="<<pb<< "ps="<<ps<<endl;
                    if (z1 < z3) {
                        mcloud->mptclout[0 + i][ps].type = 40;
                        mcloud->mptclout[0 + i][ps+1].type = 40;
                        mcloud->mptclout[0 + i][ps+2].type = 40;
                    } else {
                        mcloud->mptclout[0 + i][pb].type = 40;
                        mcloud->mptclout[0 + i][pb-1].type = 40;
                        mcloud->mptclout[0 + i][pb-2].type = 40;
                    }
                }

//                if (fabs(z2 - z4) > 0.5) {
//                    if (z2 < z4) {
//                        mcloud->mptclout[0 + i][pe].type = 100;
//                    } else {
//                        mcloud->mptclout[0 + i][pn].type = 100;
//                    }
//                }

            }
        }
    }



    float dis(float x, float y){
        float dis_=sqrt(x * x +y * y);
        return dis_;
    }


    void SpaceTimeGroundSegv2(pointCloud* mcloud, int line, int col){
        int cloud_len = mcloud->circlelen;
        //无效点不管
        if(mcloud->mptclout[line][col].type==0){
            return;
        }
        //第一条线和第二条线不管
        if(line < 2){
            return;
        }
        //与前两线的点作比较
        int selzoo = 3;
        for (int sp = col - selzoo; sp < col + selzoo; sp++)
        {
            int tp = (sp+cloud_len)%cloud_len;
            if(mcloud->mptclout[line-2][tp].type==0){
                continue;
            }
            double mptdistance = ptxydistance(mcloud->mptclout[line][col].x, mcloud->mptclout[line -2][tp].x,
                                              mcloud->mptclout[line][col].y, mcloud->mptclout[line -2][tp].y);

            if (mptdistance <lineDistance[line-2])
            {
                mcloud->mptclout[line][col].type = 20;
                break;
            }
        }
        return;
    }


    /// 清除车身周围的点
    /// \param mcloud
    void deletecarpoint(pointCloud* mcloud){
        for(int l=0;l<=3;l++){
            for(int c=0;c<mcloud->circlelen;c++){
                float x = mcloud->mptclout[l][c].x;
                float y = mcloud->mptclout[l][c].y;
                float z = mcloud->mptclout[l][c].z;
                if(fabs(x)<=1.2&& fabs(y)<=2.5){
                    mcloud->mptclout[l][c].type = 0;
                }
            }
        }
    }

    /// 点的平面间距计算
    /// \param x1
    /// \param x2
    /// \param y1
    /// \param y2
    /// \return
    double ptxydistance(float x1, float x2, float y1, float y2)
    {
        return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    }

    /// 点间空面距离计算
    /// \param p1
    /// \param p2
    /// \return
    double ptxyzdistance(pointX p1, pointX p2)
    {
        double disx = p1.x - p2.x;
        double disy = p1.y - p2.y;
        double disz = p1.z - p2.z;
        return sqrt(disx * disx + disy * disy+disz*disz );
    }

    /// 将高于地面1.8m的都变为悬空点
    /// \param mcloud
    void findtype30(pointCloud* mcloud){
        for (size_t l = 0; l < LINE; l++)
            for (size_t c = 0; c < mcloud->circlelen; c++)
            {
                if(mcloud->mptclout[l][c].type==0){
                    continue;
                }
                float z= mcloud->mptclout[l][c].z;
                float lowest = mcloud->mptclout[l][c].lowest;
                if(z-lowest > 1.8){
                    mcloud->mptclout[l][c].type=30;
                }

            }
    }

    /// 进入侦查任务时的环状珊格
    /// \param mcloud
    void circlefordetec(pointCloud *mcloud){
        mgridfac.clearGrid();

        //栅格高度统计
        for (size_t c = 0; c < mcloud->circlelen; c++)
        {
            int gridx = c / 6;
            for (size_t l = 0; l < LINE; l++)
            {
                if(mcloud->mptclout[l][c].type==0){
                    continue;
                }
                float x = mcloud->mptclout[l][c].x;
                float y = mcloud->mptclout[l][c].y;
                float z = mcloud->mptclout[l][c].z;
                float d = sqrt(x * x + y * y);
                int gridy = getgridy(d);
                mcloud->mptclout[l][c].gridx = gridx;
                mcloud->mptclout[l][c].gridy = gridy;

                if (mgridfac.mgrid[gridy][gridx].high < z)
                {
                    mgridfac.mgrid[gridy][gridx].high = z;
                }
                if (mgridfac.mgrid[gridy][gridx].low > z)
                {
                    mgridfac.mgrid[gridy][gridx].low = z;
                }

                mgridfac.mgrid[gridy][gridx].density++; //栅格中点个数
            }
        }
        //依据栅格高度提取地面点
        for (size_t l = 0; l < LINE; l++)
            for (size_t c = 0; c < mcloud->circlelen; c++)
            {
                if(mcloud->mptclout[l][c].type==0){
                    continue;
                }
                // 忽略一定高度的点

                int gridx = mcloud->mptclout[l][c].gridx;
                int gridy = mcloud->mptclout[l][c].gridy;
                double z = mcloud->mptclout[l][c].z;
                double min_z = mgridfac.mgrid[gridy][gridx].low;
                mcloud->mptclout[l][c].lowest = min_z;

                int density = 5;
                float altInCept = 0.25;

                float delta_z = z - min_z;
                if (delta_z> altInCept && mgridfac.mgrid[gridy][gridx].density >= density)
                {
                    mcloud->mptclout[l][c].type = 20;
                }
                else
                {
                    mcloud->mptclout[l][c].type = 10;
                    ///注意这里没有加反向搜索
                }
            }
    }

    /// 侦查时的地面分割接口
    /// \param mcloud
    void GroundSegForDetection(pointCloud *mcloud){
        auto startTime = std::chrono::system_clock::now();

        deletecarpoint(mcloud); //清除车身的点
        circlefordetec(mcloud); //环状栅格高度差 + 障碍物反向搜索
        //findtype30(mcloud); //悬空点寻找

        auto endTime = std::chrono::system_clock::now();
        int time = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
        printf("Time cost of GROUND-SEGMENT-detection: %d ms\n",time);
    }

    /// 正常时的地面分割接口
    /// \param mcloud
    void GroundSeg(pointCloud *mcloud)
    {
        auto startTime = std::chrono::system_clock::now();

        deletecarpoint(mcloud); //清除车身的点
        circleGroundSeg(mcloud); //环状栅格高度差 + 障碍物反向搜索
        //findtype30(mcloud); //悬空点寻找

        auto endTime = std::chrono::system_clock::now();
        int time = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
        printf("Time cost of GROUND-SEGMENT: %d ms\n",time);

    }

    void release() //释放对象
    {
        mgridfac.releaseGrid();
    }
};