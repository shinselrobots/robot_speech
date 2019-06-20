# see this webpage for info on how to create custom actions:

- https://developers.google.com/assistant/sdk/guides/service/python/extend/custom-actions
- For editing JSON, see https://jsoneditoronline.org/  (or the editor of your choice)

# Cheat Sheet:
- download gactions app (if not already in .../google_assistant_speech/resources/actions)
     https://developers.google.com/actions/tools/gactions-cli
- chmod +x gactions

-  Make Action Package accessible to the Google Assistant Server:
   - ./gactions update --action_package RobotActions.json --project robotspeechactions

-  Deploy the action package into test mode:
   - ./gactions test --action_package RobotActions.json --project robotspeechactions

