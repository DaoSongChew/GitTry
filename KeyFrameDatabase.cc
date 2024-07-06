/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "KeyFrameDatabase.h"

#include "KeyFrame.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"

#include<mutex>

using namespace std;

namespace ORB_SLAM2
{

//构造函数，参数是ORBVocabulary的引用
KeyFrameDatabase::KeyFrameDatabase (const ORBVocabulary &voc):
    mpVoc(&voc)
{
    mvInvertedFile.resize(voc.size());   //初始化mvInvertedFile（数据库），大小为ORBVocabulary的大小
}

//根据关键帧的词汇，将其添加到数据库中
void KeyFrameDatabase::add(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutex);

    for(DBoW2::BowVector::const_iterator vit= pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit!=vend; vit++)
        mvInvertedFile[vit->first].push_back(pKF);
}

//从数据库中删除对应的关键帧
void KeyFrameDatabase::erase(KeyFrame* pKF)
{
    unique_lock<mutex> lock(mMutex);

    // Erase elements in the Inverse File for the entry
    for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit!=vend; vit++)   //遍历关键帧的词汇
    {
        // List of keyframes that share the word
        list<KeyFrame*> &lKFs =   mvInvertedFile[vit->first];   //获取关键帧的词汇在数据库中的列表

        for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
        {
            if(pKF==*lit)   //如果找到了要删除的关键帧，则从列表中删除
            {
                lKFs.erase(lit);    
                break;
            }
        }
    }
}

//清空关键帧数据库
void KeyFrameDatabase::clear()
{
    mvInvertedFile.clear();
    mvInvertedFile.resize(mpVoc->size());
}


vector<KeyFrame*> KeyFrameDatabase::DetectLoopCandidates(KeyFrame* pKF, float minScore) //检测关键帧pKF与其他关键帧是否存在循环关系，用于闭环检测，有两个参数，minScore是最小的相似度分数，返回的是与pKF有连接关系的关键帧的列表
{
    set<KeyFrame*> spConnectedKeyFrames = pKF->GetConnectedKeyFrames();     //用于存储与pKF有连接关系的关键帧
    list<KeyFrame*> lKFsSharingWords;    //用于存储与pKF有共享词汇的关键帧，并且和pKF没有连接关系

    // Search all keyframes that share a word with current keyframes
    // Discard keyframes connected to the query keyframe
    {
        unique_lock<mutex> lock(mMutex);

        for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit != vend; vit++)     //遍历关键帧的辞典，辞典的键单词，值是关键帧指针的列表
        {
            list<KeyFrame*> &lKFs =   mvInvertedFile[vit->first];       //获得共享单词的关键帧的列表

            for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++) //对共享词汇的关键帧进行遍历
            {
                KeyFrame* pKFi=*lit;        //共享词汇的关键帧
                if(pKFi->mnLoopQuery!=pKF->mnId)    //如果共享单词的关键帧的mnLoopQuery不是pKF的ID，则更新mnLoopWords，也就是要排除调自身
                {
                    pKFi->mnLoopWords=0;    //初始化mnLoopWords为0
                    if(!spConnectedKeyFrames.count(pKFi))       //如果该关键帧不是pKF的连接关键帧，则更新mnLoopWords
                    {
                        pKFi->mnLoopQuery=pKF->mnId;    //更新PKFi的mnLoopQuery为pKF的ID
                        lKFsSharingWords.push_back(pKFi);   //将共享词汇的关键帧加入到lKFsSharingWords中
                    }
                }
                pKFi->mnLoopWords++;
            }
        }
    }

    if(lKFsSharingWords.empty())    //如果没有找到与pKF有共享词汇的关键帧，则返回空列表
        return vector<KeyFrame*>();

    list<pair<float,KeyFrame*> > lScoreAndMatch;    

    // Only compare against those keyframes that share enough words
    int maxCommonWords=0;
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++) //遍历共享词汇且没有连接关系的关键帧
    {
        if((*lit)->mnLoopWords>maxCommonWords)      //找到最大的mnLoopWords，即最大的共享单词数
            maxCommonWords=(*lit)->mnLoopWords;
    }

    int minCommonWords = maxCommonWords*0.8f;    //设置最小的共享单词数，即至少要有80%的最大共享单词数才可以作为候选
    int nscores=0;

    // Compute similarity score. Retain the matches whose score is higher than minScore
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)     //遍历共享词汇且没有连接关系的关键帧
    {
        KeyFrame* pKFi = *lit;

        if(pKFi->mnLoopWords>minCommonWords)     //如果关键帧的mnLoopWords大于最小的共享单词数，则计算相似度分数
        {
            nscores++;  //统计数量

            float si = mpVoc->score(pKF->mBowVec,pKFi->mBowVec);    //计算两个关键帧的词汇相似度得分

            pKFi->mLoopScore = si;
            if(si>=minScore)
                lScoreAndMatch.push_back(make_pair(si,pKFi));
        }
    }

    if(lScoreAndMatch.empty())      //如果没有找到相似度分数大于minScore的关键帧，则返回空列表，没有能够作为候选的关键帧
        return vector<KeyFrame*>();

    list<pair<float,KeyFrame*> > lAccScoreAndMatch;
    float bestAccScore = minScore;

    // Lets now accumulate score by covisibility
    for(list<pair<float,KeyFrame*> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)  //遍历候选关键帧列表
    {
        KeyFrame* pKFi = it->second;
        vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);    //获得与候选关键帧有最好的公式关系排名前十的关键帧（二级）

        float bestScore = it->first;
        float accScore = it->first;
        KeyFrame* pBestKF = pKFi;
        for(vector<KeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)    //遍历二级关键帧
        {
            KeyFrame* pKF2 = *vit;
            if(pKF2->mnLoopQuery==pKF->mnId && pKF2->mnLoopWords>minCommonWords)        //如果二级关键帧的mnLoopQuery是pKF的ID，并且mnLoopWords大于最小的共享单词数
            {
                accScore+=pKF2->mLoopScore;
                if(pKF2->mLoopScore>bestScore)
                {
                    pBestKF=pKF2;
                    bestScore = pKF2->mLoopScore;
                }
            }
        }

        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
        if(accScore>bestAccScore)
            bestAccScore=accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f*bestAccScore;

    set<KeyFrame*> spAlreadyAddedKF;    //定义一个集合，用来存放已经添加到vpLoopCandidates中的关键帧指针
    vector<KeyFrame*> vpLoopCandidates;    //定义一个向量，用来存放回环的候选关键帧指针
    vpLoopCandidates.reserve(lAccScoreAndMatch.size());    //预分配空间

    for(list<pair<float,KeyFrame*> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)    //遍历lAccScoreAndMatch中的关键帧指针
    {
        if(it->first>minScoreToRetain)    //如果分数大于阈值minScoreToRetain
        {
            KeyFrame* pKFi = it->second;
            if(!spAlreadyAddedKF.count(pKFi))
            {
                vpLoopCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }


    return vpLoopCandidates;        //返回PKF的回环候选关键帧列表
}

vector<KeyFrame*> KeyFrameDatabase::DetectRelocalizationCandidates(Frame *F)    //根据当前帧F与数据库检测是否和其他帧存在重定位关系，用于重定位
{
    //搜索所有和F有着相同单词的关键帧存储在lKFsSharingWords中
    list<KeyFrame*> lKFsSharingWords;   

    // Search all keyframes that share a word with current frame
    //更新关键帧中的mnRelocWords表示和F有多少词汇相同
    {
        unique_lock<mutex> lock(mMutex);

        for(DBoW2::BowVector::const_iterator vit=F->mBowVec.begin(), vend=F->mBowVec.end(); vit != vend; vit++)
        {
            list<KeyFrame*> &lKFs =   mvInvertedFile[vit->first];   //获取关键帧的词汇在数据库中的列表，结果是关键帧指针的列表

            for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                KeyFrame* pKFi=*lit;        //获取关键帧指针
                if(pKFi->mnRelocQuery!=F->mnId) //如果该关键帧不是来自当前帧，则更新mnRelocWords
                {
                    pKFi->mnRelocWords=0;    //初始化mnRelocWords为0，用来记录和当前帧有多少词汇相同
                    pKFi->mnRelocQuery=F->mnId;  //更新mnRelocQuery为当前帧的ID
                    lKFsSharingWords.push_back(pKFi);    //将该关键帧指针加入到lKFsSharingWords中
                }
                pKFi->mnRelocWords++;    //更新mnRelocWords
            }
        }
    }
    if(lKFsSharingWords.empty())    //如果lKFsSharingWords为空，则返回空的结果
        return vector<KeyFrame*>(); //返回空的结果

    // Only compare against those keyframes that share enough words
    int maxCommonWords=0;    //获取lKFsSharingWords中关键帧中词汇最多的关键帧
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)        //遍历lKFsSharingWords中的关键帧指针
    {
        if((*lit)->mnRelocWords>maxCommonWords)
            maxCommonWords=(*lit)->mnRelocWords;    //获取共享单词的关键帧中，最多的共享单词数量
    }

    int minCommonWords = maxCommonWords*0.8f;    //获取词汇最多的关键帧的词汇最少的比例

    list<pair<float,KeyFrame*> > lScoreAndMatch;    //定义一个列表，用来存放关键帧的分数和关键帧指针

    int nscores=0;

    // Compute similarity score.
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++) //遍历lKFsSharingWords中的关键帧，当其中的mRelocScore大于阈值minCommonWords时，将其加入到lScoreAndMatch中
    {
        KeyFrame* pKFi = *lit;

        if(pKFi->mnRelocWords>minCommonWords)
        {
            nscores++;      //共享单词数量大于阈值minCommonWords的关键帧的数量
            float si = mpVoc->score(F->mBowVec,pKFi->mBowVec);  //计算当前帧和关键帧的词汇相似度
            pKFi->mRelocScore=si;   //更新关键帧的mRelocScore得分
            lScoreAndMatch.push_back(make_pair(si,pKFi));
        }
    }

    if(lScoreAndMatch.empty())    //如果lScoreAndMatch为空，则返回空的结果
        return vector<KeyFrame*>();

    list<pair<float,KeyFrame*> > lAccScoreAndMatch;
    float bestAccScore = 0;

    // Lets now accumulate score by covisibility
    for(list<pair<float,KeyFrame*> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)    //遍历lScoreAndMatch中的关键帧指针
    {
        KeyFrame* pKFi = it->second;    //获取符合条件的关键帧指针
        vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);    //获得与该关键帧由最好的共视程度排名前十的邻居向量

        float bestScore = it->first;    //获得与当前关键帧的词汇相似度得分
        float accScore = bestScore;
        KeyFrame* pBestKF = pKFi;

        for(vector<KeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)    //对邻居关键帧进行遍历，获取最佳邻居关键帧
        {
            KeyFrame* pKF2 = *vit;
            if(pKF2->mnRelocQuery!=F->mnId)     //说明PKF2与F没有共同的词汇，就此放弃
                continue;

            accScore+=pKF2->mRelocScore;        //在原来的满足共享单词数阈值的关键帧的得分基础上，再加上该帧的共视程度最好的十个邻居关键帧和当前帧的词汇相似度得分
            if(pKF2->mRelocScore>bestScore) //如果邻居关键帧的词汇相似度得分大于当前关键帧的词汇相似度得分
            {
                pBestKF=pKF2;
                bestScore = pKF2->mRelocScore;
            }

        }
        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));   //
        if(accScore>bestAccScore)       //获得最高的得分
            bestAccScore=accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    ////返回lAccScoreAndMatch中所有的分超过0.75*bestAccScore的keyframe集合
    float minScoreToRetain = 0.75f*bestAccScore;    //定义分数阈值
    set<KeyFrame*> spAlreadyAddedKF;    //定义一个集合，用来存放已经添加到vpRelocCandidates中的关键帧指针
    vector<KeyFrame*> vpRelocCandidates;    //定义一个向量，用来存放重定位的待选关键帧指针
    vpRelocCandidates.reserve(lAccScoreAndMatch.size());    //预分配空间
    for(list<pair<float,KeyFrame*> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)    //遍历lAccScoreAndMatch中的关键帧指针
    {
        const float &si = it->first;    //获取关键帧的词汇相似度得分
        if(si>minScoreToRetain)         //如果分数大于阈值minScoreToRetain
        {
            KeyFrame* pKFi = it->second;    //获取关键帧指针
            if(!spAlreadyAddedKF.count(pKFi))   //如果该关键帧指针没有被添加到vpRelocCandidates中
            {
                vpRelocCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }

    return vpRelocCandidates;       //返回重定位的待选关键帧集合
}

} //namespace ORB_SLAM
