#include "PathPlanner.h"



PathPlanner::PathPlanner():
startSet(false),endSet(false),startPoint(),endPoint()
{
    startSet = false;
    endSet = false;


}

PathPlanner::PathPlanner(Eigen::Vector2i startPointIn,Eigen::Vector2i endPointIn):
startSet(true),endSet(true),startPoint(startPointIn),endPoint(endPointIn)
{


}

bool PathPlanner::isReady(){
    return (startSet && endSet);
}


void PathPlanner::calculatePath(){
    if(startSet && endSet){
        //TODO: A*
    }
    else{
        return;
    }
}


void PathPlanner::rasterWorld(World& world){

    float height = floor(world.worldVerticies.getBounds().height);
    float width = floor(world.worldVerticies.getBounds().width);
    float y0 = floor(world.worldVerticies.getBounds().top);
    float x0 = floor(world.worldVerticies.getBounds().left);
    float y1 = floor(height + y0);
    float x1 = floor(width  + x0);
    Eigen::MatrixXf raster = Eigen::MatrixXf::Ones(width,height); //1 is clear 0 is not 

    for(int z = 1; z < world.circles.size(); z++){
        float cx = world.circles[z].getPosition().x;
        float cy = world.circles[z].getPosition().y;
        float rad = world.circles[z].getRadius();

        for(int i =floor(cx-rad);i<floor(cx+rad);i++ ){
            for(int j = floor(cy-rad); j<floor(cy+rad);j++){
                if(circleCol(cx,cy,i,j,rad)) raster(i,j) = 0;
                
            }
        }
    }

}
/*
    Checks for collision between a circle at cx,cy with radius rad
    and a square with top left corner cx,cy and width/length 1
*/
bool PathPlanner::circleCol(float cx, float cy, int x, int y,float rad){

    float testX = cx;
    float testY = cy;
    if(cx < x) testX = x;
    else if(cx > x+1) testX = x+1;
    if(cy < y) testY = y;
    else if(cy > y+1) testY = y+1;
    float dist = (cx-testX)*(cx-testX) + (cy-testY)*(cy-testY);

    return (dist <= rad*rad);
}