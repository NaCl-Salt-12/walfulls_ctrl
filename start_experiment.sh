#!/bin/bash

set -e
# PATH TO YOUR PRIVATE DEPLOY KEY (REQUIRED)
# IMPORTANT: This must be the path to the private key used for GitHub deployment.
DEPLOY_KEY_PATH="/home/ciscor/.ssh/id_ed25519"

# Set permissions for the private key (should be 600 or 400)
chmod 600 "$DEPLOY_KEY_PATH"

if [ ! -f "$DEPLOY_KEY_PATH" ]; then
	echo "ERROR: Deploy key file not found at: $DEPLOY_KEY_PATH"
	echo "Please ensure the path is correct and the file has 400 or 600 permissions."
	exit 1
fi
# Exit immediately if a command exits with a non-zero status.

echo "Starting experiment setup..."

# --- 1. Determine Experiment Name ---
python3 ./scripts/get_experiment_name.py

EXPERIMENT_NAME=$(cat "${HOME}/.experiment_name")

if [ -z "$EXPERIMENT_NAME" ]; then
	echo "ERROR: Python script failed to generate an experiment name."
	exit 1
fi

echo "Experiment name determined: ${EXPERIMENT_NAME}"

rm "${HOME}/.experiment_name"

TIMESTAMP=$(date +"%Y%m%d_%H%M")
EXPERIMENT_NAME_FULL="${TIMESTAMP}_${EXPERIMENT_NAME}"

# --- 3. Experiment Launch ---
echo "Running experiment: ${EXPERIMENT_NAME}"
ros2 launch launch/experiment_launch.py experiment_name:="${EXPERIMENT_NAME_FULL}"

BAG_FOLDER="${HOME}/bag_data/${EXPERIMENT_NAME_FULL}"

# --- 5. CSV Conversion and Renaming ---

# Convert the bag file to CSVs
python3 ./scripts/rosbag2csv.py "$BAG_FOLDER"

# Rename the generated CSV files
for file in "${BAG_FOLDER}"/*.csv; do
	# Check if a file was actually found
	if [ -f "$file" ]; then
		file_name=$(basename "$file")
		new_name="${BAG_FOLDER}/${EXPERIMENT_NAME}_${file_name}"

		echo "Renaming '$file' to '$new_name'"
		mv "$file" "$new_name"
	fi
done

DIR="${HOME}/experiment_logs"

if [[ ! -d "$DIR" ]]; then
	echo "ERROR: Directory $DIR does not exist."
	exit 1
fi

cp -r "$BAG_FOLDER" "$DIR"

cd "$DIR"

if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
	echo "ERROR: Not currently inside a Git repository."
	exit 1
fi

BRANCH_NAME="main"
TIMESTAMP2=$(date +"%Y-%m-%d %H:%M:%S")
COMMIT_MESSAGE="Experiment ${EXPERIMENT_NAME} completed at ${TIMESTAMP2}"

git remote set-url origin git@github.com:Optimal-Robotics-Lab/wafulls_boom_data.git

# Verify the change (optional)
echo "Origin remote set to:"
git remote -v

# Stage changes
git add .
echo "Staged all changes (git add .)"

# Commit staged changes
if git commit -m "$COMMIT_MESSAGE"; then
	echo "Successfully committed changes: \"$COMMIT_MESSAGE\""
else
	# Check if the commit failed because there were no changes
	if git status --porcelain | grep -q '^\?'; then
		# This means there were untracked files, but no tracked file changes. Commit still failed.
		echo "ERROR: Commit failed. Check git status."
		exit 1
	else
		# No actual changes to commit (tracked files are clean)
		echo "No changes detected. Skipping commit and push."
		exit 0
	fi
fi

# Push using the specific deploy key
echo "Pushing to origin/$BRANCH_NAME via SSH..."

# Use GIT_SSH_COMMAND to force SSH to use the specific key file.
# -i <path>: Specifies the identity (key) file.
# -o IdentitiesOnly=yes: Prevents SSH from attempting to use other keys in the agent.
# -o StrictHostKeyChecking=no: Avoids being prompted to confirm the host key on first connect
GIT_SSH_COMMAND="ssh -i $DEPLOY_KEY_PATH -o IdentitiesOnly=yes -o StrictHostKeyChecking=no" git push origin "$BRANCH_NAME"

# Check the exit status of the push command
if [ $? -eq 0 ]; then
	echo "DEPLOYMENT SUCCESSFUL!"
else
	echo "DEPLOYMENT FAILED during push."
	exit 1
fi

# Clean up environment variable
unset GIT_SSH_COMMANDxit 0
