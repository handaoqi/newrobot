#!/usr/bin/env bash
set -euo pipefail

frontend_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
release_version=${RELEASE_VERSION:-v1.0.0}
image="roamerx/platform-frontend:${release_version}-amd64"
output_dir=${RELEASE_OUTPUT_DIR:-"${frontend_dir}/release-out"}
package_name="roamerx-frontend-${release_version}-linux-amd64"
package_dir="${output_dir}/${package_name}"
build_time=$(date -u +%Y-%m-%dT%H:%M:%SZ)
git_revision=$(git -C "${frontend_dir}/../.." rev-parse HEAD 2>/dev/null || printf 'unknown')

rm -rf "${package_dir}"
mkdir -p "${package_dir}/nginx"

docker build --platform linux/amd64 \
  --build-arg NODE_IMAGE="${NODE_IMAGE:-node:22-alpine}" \
  --build-arg NGINX_IMAGE="${NGINX_IMAGE:-nginx:1.27-alpine}" \
  --label org.opencontainers.image.version="${release_version}" \
  --label org.opencontainers.image.revision="${git_revision}" \
  --label org.opencontainers.image.created="${build_time}" \
  -t "${image}" "${frontend_dir}"
docker image inspect "${image}" --format '{{.Architecture}}' | grep -Fx amd64
docker save "${image}" -o "${package_dir}/frontend-image.tar"

cp "${frontend_dir}/compose.frontend.yml" "${package_dir}/compose.frontend.yml"
cp "${frontend_dir}/modules.json" "${package_dir}/modules.json"
cp "${frontend_dir}/release/frontendctl" "${package_dir}/frontendctl"
cp "${frontend_dir}/release/README.md" "${package_dir}/README.md"
cp "${frontend_dir}/release/nginx/"* "${package_dir}/nginx/"
cat > "${package_dir}/release.json" <<EOF
{
  "release_version": "${release_version}",
  "package": "${package_name}",
  "image": "${image}",
  "architecture": "amd64",
  "git_revision": "${git_revision}",
  "built_at": "${build_time}",
  "node_base": "${NODE_IMAGE:-node:22-alpine}",
  "nginx_base": "${NGINX_IMAGE:-nginx:1.27-alpine}"
}
EOF
chmod +x "${package_dir}/frontendctl"
tar -C "${output_dir}" -czf "${output_dir}/${package_name}.tar.gz" "${package_name}"
sha256sum "${output_dir}/${package_name}.tar.gz" > "${output_dir}/SHA256SUMS"
printf 'created %s\n' "${output_dir}/${package_name}.tar.gz"
