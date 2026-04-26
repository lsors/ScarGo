#pragma once

#include "esp_http_server.h"

esp_err_t ota_service_handle_upload(httpd_req_t *req);
