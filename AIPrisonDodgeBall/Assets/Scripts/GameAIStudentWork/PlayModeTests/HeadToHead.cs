﻿
using System.IO;
using System.Reflection;
using System.Collections;
using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;

using UnityEngine.SceneManagement;
using UnityEngine.TestTools;


namespace Tests
{

    public class HeadToHead
    {
        const int matchLenSec = 120;
        const int numMatches = 10;
        const int timeScale = 1; // how fast to run the game. Running fast doesn't necessarily
                                 // give accurate results.

        const int PlayMatchTimeOutMS = int.MaxValue; // don't mess with this; add it to new tests
                                                     // as [Timeout(PlayMatchTimeOutMS)] (see below for
                                                     // example) It stops early default timeout

        public HeadToHead()
        {

        }


              // 2v2
        [UnityTest]
        [Timeout(PlayMatchTimeOutMS)]
        public IEnumerator HeadToHead_2v2_1b()
        {
            return TestMatch(2, 1);
        }

        [UnityTest]
        [Timeout(PlayMatchTimeOutMS)]
        public IEnumerator HeadToHead_2v2_2b()
        {
            return TestMatch(2, 2);
        }

        [UnityTest]
        [Timeout(PlayMatchTimeOutMS)]
        public IEnumerator HeadToHead_2v2_3b()
        {
            return TestMatch(2, 3);
        }

        [UnityTest]
        [Timeout(PlayMatchTimeOutMS)]
        public IEnumerator HeadToHead_2v2_4b()
        {
            return TestMatch(2, 4);
        }

        // 3v3
        [UnityTest]
        [Timeout(PlayMatchTimeOutMS)]
        public IEnumerator HeadToHead_3v3_1b()
        {
            return TestMatch(3, 1);
        }

        [UnityTest]
        [Timeout(PlayMatchTimeOutMS)]
        public IEnumerator HeadToHead_3v3_2b()
        {
            return TestMatch(3, 2);
        }

        [UnityTest]
        [Timeout(PlayMatchTimeOutMS)]
        public IEnumerator HeadToHead_3v3_3b()
        {
            return TestMatch(3, 3);
        }

        [UnityTest]
        [Timeout(PlayMatchTimeOutMS)]
        public IEnumerator HeadToHead_3v3_4b()
        {
            return TestMatch(3, 4);
        }

        // 4v4
        [UnityTest]
        [Timeout(PlayMatchTimeOutMS)]
        public IEnumerator HeadToHead_4v4_1b()
        {
            return TestMatch(4, 1);
        }

        [UnityTest]
        [Timeout(PlayMatchTimeOutMS)]
        public IEnumerator HeadToHead_4v4_2b()
        {
            return TestMatch(4, 2);
        }

        [UnityTest]
        [Timeout(PlayMatchTimeOutMS)]
        public IEnumerator HeadToHead_4v4_3b()
        {
            return TestMatch(4, 3);
        }

        [UnityTest]
        [Timeout(PlayMatchTimeOutMS)]
        public IEnumerator HeadToHead_4v4_4b()
        {
            return TestMatch(4, 4);
        }

        // 5v5
        [UnityTest]
        [Timeout(PlayMatchTimeOutMS)]
        public IEnumerator HeadToHead_5v5_1b()
        {
            return TestMatch(5, 1);
        }

        [UnityTest]
        [Timeout(PlayMatchTimeOutMS)]
        public IEnumerator HeadToHead_5v5_2b()
        {
            return TestMatch(5, 2);
        }

        [UnityTest]
        [Timeout(PlayMatchTimeOutMS)]
        public IEnumerator HeadToHead_5v5_3b()
        {
            return TestMatch(5, 3);
        }

        [UnityTest]
        [Timeout(PlayMatchTimeOutMS)]
        public IEnumerator HeadToHead_5v5_4b()
        {
            return TestMatch(5, 4);
        }


        [Timeout(PlayMatchTimeOutMS)]
        public IEnumerator TestMatch(
            int teamSize,
            int ballsPerTeam
            )
        {
            Time.timeScale = timeScale;

            int numWins = 0;
            int numLosses = 0;
            int numTies = 0;

            Debug.Log($"TESTING dodgeball: teamSize={teamSize} ballsPerTeam={ballsPerTeam}");
            Debug.Log($"Total matches for test: {numMatches} Match Len Sec: {matchLenSec}");

            // Pick your agents for the matchup
            var playerTeamName = "MinionStateMachine, GameAIStudentWork, Version=0.0.0.0, Culture=neutral, PublicKeyToken=null";
            var opponentTeamName = "MinionBasicDemoStateMachine, GameAIFSM, Version=0.0.0.0, Culture=neutral, PublicKeyToken=null";

            for (int i = 0; i < numMatches; ++i)
            {
                var sceneName = "PrisonBall";

                bool isTeamAThePlayer = i % 2 == 0;

                PrisonDodgeballManager.OverrideConfiguration = true;
                PrisonDodgeballManager.Override_TeamAAssemblyQualifiedName = isTeamAThePlayer ? playerTeamName : opponentTeamName;
                PrisonDodgeballManager.Override_TeamBAssemblyQualifiedName = isTeamAThePlayer ? opponentTeamName : playerTeamName;
                PrisonDodgeballManager.Override_teamSize = teamSize;
                PrisonDodgeballManager.Override_ballsPerTeam = ballsPerTeam;
                PrisonDodgeballManager.Override_matchLengthSec = matchLenSec;

                SceneManager.LoadScene(sceneName);

                var waitForScene = new WaitForSceneLoaded(sceneName);
                yield return waitForScene;

                Assert.IsFalse(waitForScene.TimedOut, "Scene " + sceneName + " was never loaded");

                PrisonDodgeballManager mgr = PrisonDodgeballManager.Instance;

                var waitForMatchEnd = new WaitForCondition(() => mgr.IsGameOver, PrisonDodgeballManager.Override_matchLengthSec + 5);

                yield return waitForMatchEnd;

                Assert.IsFalse(waitForMatchEnd.TimedOut, "Match never ended");

                PrisonDodgeballManager.Team playersTeam = isTeamAThePlayer ? PrisonDodgeballManager.Team.TeamA : PrisonDodgeballManager.Team.TeamB;

                string status = "lost";
                if (mgr.IsTie)
                {
                    status = "tied";
                    numTies += 1;
                }
                else if (mgr.IsWinner(playersTeam))
                {
                    status = "won";
                    numWins += 1;
                }
                else
                {
                    status = "lost";
                    numLosses += 1;
                }

                Debug.Log($"Player's team {status}! Now at Win-Loss-Tie: {numWins}-{numLosses}-{numTies}");

            } //for

            var winRatio = numWins / (float)(numWins + numLosses + numTies);
            var winTarget = 2f / 3f;
            Assert.That(winRatio, Is.GreaterThanOrEqualTo(winTarget));

        }

    }

}

