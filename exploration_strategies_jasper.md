## Next Steps
# Convert map points to coordinates
- done
# Determine frontiers (between explore/un-explored pixels)
- done
# Give scores to each frontier pixel based on proximity to other frontiers
- done
# Find highest scoring frontier
- 
- Adjust waypoint_cycler.py to navigate between random frontiers for now
- Adjust strategy as you debug

## When exploration fails...
- can subscribe to BehaviourTreeLog's NavigateRecovery. when this is 'IDLE' either achieved goal pose or experienced a planning or execution failure
- 
