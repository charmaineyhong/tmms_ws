# C2 storage backend (app/tmms_c2). MongoDB itself runs from the stock mongo:7 image next to
# it; see app/tmms_c2/tmms_c2-compose.yaml.
#
# No RUN steps on purpose: node_modules is installed on the host and copied in. Its packages
# are plain JavaScript (no compiled .node files), so the same folder works on the robot's arm64,
# and an x86 laptop can build the arm64 image without emulating arm64.
#
# Build for the robot from the repo root (context is app/tmms_c2, not the whole repo):
#   (cd app/tmms_c2 && npm ci --omit=dev)
#   docker buildx build --platform linux/arm64 -f devops/tmms_c2.Dockerfile -t tmms_c2_image:0.1.0 --load app/tmms_c2
FROM node:22-slim

ENV NODE_ENV=production
WORKDIR /app

COPY --chown=node:node node_modules ./node_modules
COPY --chown=node:node package.json c2_backend.js ./

USER node

EXPOSE 3002
CMD ["node", "c2_backend.js"]
