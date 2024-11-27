# RTOS-Project

Summary of Efforts to Date:
I have completed 100% of my expected work (36.5 hr out of 36.5 hr) in 90.8% of my budgeted time (33.15 hr out of 36.5 hr). For the work that has been completed, I took 0.908x the time I expected. (Note that although I completed all the tasks in my tasks table last week, I added a little more time this week in an attempt to improve my solution.)

Analysis of Estimated Effort vs. Actual for "Update position task for hitting walls" task: I expected this item to take 1 hour, and it took 7 instead. I believe this is because when I made my estimate, I had no plan or even basic idea for how I would implement it (whereas many other tasks were easier to estimate because they were very similar to things we've done in previous labs and I knew what would be required). When I got to this task, I took awhile just figuring out what to do, drawing it out, experimenting with different ideas and ways of storing the map. I then got it working, except for an edge case in which the ball would go through a corner if there was no connected edge adjacent to the ball's current cell (which isn't easy to take advantage of but happens often enough to be noticed). Later on, I went back and worked through this problem, which took some time again to figure out a good implementation. The user can now play it either with the buggy corners (let CHECK_CORNERS=0) and try to use that to teleport through corners without the disruptor, or with the ball hitting corners (let CHECK_CORNERS=1).

Summary of Project at End of Semester:
This week I added in the waypoint reuse mode functionality, and I allowed the numbers of cells and waypoints to be configured in the application header file. The game is now fully playable and has a good amount of options (all in the header file), including game mode (waypoint reuse on/off, hard-edged on/off, pin-maze on/off), physics (angle gain, gravity, and ball throwing difficulty), the maximum time for the game and the disruptor, the disruptor energy levels and rates, the map (number of cells, probability of walls, number of holes/waypoints), and the frequencies at which different tasks are run (not recommended to change). The waypoints are color-coded for the ordering (yellow is the current target, red will be next, and the others display in magenta). The end-of-game screen displays with the time, the number of waypoints (in the case of a win), and the score.

