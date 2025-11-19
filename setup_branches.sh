#!/bin/bash
# Script to safely create branches for your DMX controller project

set -e  # Exit on any error

cd ~/pi-dmx-controller

echo "=== Current Git Status ==="
git status

echo ""
echo "=== Current Branch ==="
git branch --show-current

echo ""
read -p "Do you have uncommitted changes you want to commit? (y/n): " commit_now

if [ "$commit_now" = "y" ]; then
    echo ""
    echo "Adding all changes..."
    git add -A
    echo ""
    read -p "Enter commit message: " commit_msg
    git commit -m "$commit_msg"
fi

echo ""
echo "=== Creating 'stable' branch as backup of current working version ==="
git branch stable 2>/dev/null || echo "Branch 'stable' already exists"
git push origin stable

echo ""
echo "=== Creating 'experimental-presets' branch for your new changes ==="
git checkout -b experimental-presets 2>/dev/null || git checkout experimental-presets

echo ""
echo "=== Setup Complete! ==="
echo "You are now on branch: $(git branch --show-current)"
echo ""
echo "Branch overview:"
echo "  - 'main' = your original branch"
echo "  - 'stable' = backup/snapshot of your working version"
echo "  - 'experimental-presets' = where you'll make new changes (CURRENT)"
echo ""
echo "To switch branches:"
echo "  git checkout main                  # back to original"
echo "  git checkout stable                # to stable backup"
echo "  git checkout experimental-presets  # to experimental (current)"



