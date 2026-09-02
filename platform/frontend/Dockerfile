ARG NODE_IMAGE=node:22-alpine
ARG NGINX_IMAGE=nginx:1.27-alpine
ARG VITE_BUILD_TARGET=container
FROM ${NODE_IMAGE} AS build

ARG VITE_BUILD_TARGET
ENV VITE_BUILD_TARGET=${VITE_BUILD_TARGET}

WORKDIR /app
COPY package*.json ./
RUN npm config set registry https://registry.npmmirror.com && npm ci
COPY . .
RUN npm run build

FROM ${NGINX_IMAGE}

COPY nginx.conf /etc/nginx/conf.d/default.conf
COPY --from=build /app/dist /usr/share/nginx/html

EXPOSE 80
