#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "avt_341_nav/communication/socket_message_framer.h"

using avt_341_nav::communication::SocketMessageFramer;
using Messages = std::vector<std::string>;

TEST(SocketMessageFramer, UnterminatedReadsAreWholeMessages)
{
    SocketMessageFramer framer;
    EXPECT_EQ(framer.Feed("MRZR,0,SET_SPEED,MRZR,5"), Messages({"MRZR,0,SET_SPEED,MRZR,5"}));
    EXPECT_EQ(framer.Feed("MRZR,1,CANCEL_ALL,MRZR,QUEUE"), Messages({"MRZR,1,CANCEL_ALL,MRZR,QUEUE"}));
    EXPECT_TRUE(framer.Feed("").empty());
}

TEST(SocketMessageFramer, GluedMessagesAreSeparated)
{
    SocketMessageFramer framer;
    EXPECT_EQ(framer.Feed("MRZR,0,MOVETO,MRZR,MP_Y,0,0,0,-1,QUEUE,\nFEDA,4,MOVETO,FEDA,MP_Z,0,0,0,-1,QUEUE,car\n"),
              Messages({"MRZR,0,MOVETO,MRZR,MP_Y,0,0,0,-1,QUEUE,", "FEDA,4,MOVETO,FEDA,MP_Z,0,0,0,-1,QUEUE,car"}));
}

TEST(SocketMessageFramer, SplitMessageIsReassembled)
{
    SocketMessageFramer framer;
    EXPECT_EQ(framer.Feed("MRZR,0,SET_SPEED,MRZR,5\nMRZR,1,MOVETO,MRZR,MP_Y,0,0,0,-1,QUEUE,^T"),
              Messages({"MRZR,0,SET_SPEED,MRZR,5"}));
    EXPECT_TRUE(framer.Feed("I_|ca").empty());
    EXPECT_EQ(framer.Feed("r\n"), Messages({"MRZR,1,MOVETO,MRZR,MP_Y,0,0,0,-1,QUEUE,^TI_|car"}));
}

TEST(SocketMessageFramer, CarriageReturnsAndEmptyLinesAreHarmless)
{
    SocketMessageFramer framer;
    EXPECT_EQ(framer.Feed("MRZR,0,SET_SPEED,MRZR,5\r\n\nMRZR,1,CANCEL_ALL,MRZR,QUEUE\r\n"),
              Messages({"MRZR,0,SET_SPEED,MRZR,5\r", "MRZR,1,CANCEL_ALL,MRZR,QUEUE\r"}));
}

TEST(SocketMessageFramer, FrameAppendsDelimiter)
{
    EXPECT_EQ(SocketMessageFramer::Frame("MRZR,0,SET_SPEED,MRZR,5"), "MRZR,0,SET_SPEED,MRZR,5\n");
}
