//
// Created by Puyang Wang on 4/15/16.
//

#ifndef TRACK_CHESSBOARD_HPP
#define TRACK_CHESSBOARD_HPP

glm::mat4 getViewMatrix();
bool loadIntrinsics(const char * K_path,const char * D_path);

bool track_chessboard(Mat ImgOrigin);

#endif //OPENGL_STUDY_TRACK_CHESSBOARD_H
