import copy
import numpy as np

def tensorflow_run(self, nei_pos, nei_dis):
    if(self.isBeacon == True):
            return
    if self.nei_id == []:
        print('neighbor is none')
        return
    if self.isFinalPos == True:
        return
    # print('self.nei_id is',self.nei_id)
    # print('self.nei_dis is ', nei_dis)
    for i in range(self.epoch):
        coord, _, loss = self.sess.run([
            self.coord,
            self.train_op,
            self.reduced_loss
        ], feed_dict={
            self.neighborsCoord: nei_pos,
            self.dist_gt: nei_dis
        })
    self.loss_dump.append(copy.deepcopy(loss))
    print("loss : ", loss)
