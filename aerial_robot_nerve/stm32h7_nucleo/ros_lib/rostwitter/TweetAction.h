#ifndef _ROS_rostwitter_TweetAction_h
#define _ROS_rostwitter_TweetAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "rostwitter/TweetActionGoal.h"
#include "rostwitter/TweetActionResult.h"
#include "rostwitter/TweetActionFeedback.h"

namespace rostwitter
{

  class TweetAction : public ros::Msg
  {
    public:
      typedef rostwitter::TweetActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef rostwitter::TweetActionResult _action_result_type;
      _action_result_type action_result;
      typedef rostwitter::TweetActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    TweetAction():
      action_goal(),
      action_result(),
      action_feedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->action_goal.serialize(outbuffer + offset);
      offset += this->action_result.serialize(outbuffer + offset);
      offset += this->action_feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->action_goal.deserialize(inbuffer + offset);
      offset += this->action_result.deserialize(inbuffer + offset);
      offset += this->action_feedback.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "rostwitter/TweetAction"; };
    const char * getMD5(){ return "2fbb8e01060b10f950ec4190c26ea523"; };

  };

}
#endif
