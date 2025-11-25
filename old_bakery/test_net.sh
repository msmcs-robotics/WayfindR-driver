#!/usr/bin/env bash

### CONFIG ###############################################################
IFACE="wlx782051096933"
LOG_DIR="${HOME}/logs"
LOG_FILE="${LOG_DIR}/net_errs.txt"
REQUIRED_TOOLS=(ip ping dig curl)
OPTIONAL_TOOLS=(traceroute ufw iptables)
TEST_HOST="google.com"
TEST_IP="8.8.8.8"
##########################################################################

ERRORS=0
LOGGING=false

# --- detect argument
if [[ "$1" == "--errors" ]]; then
    LOGGING=true
fi

# --- timestamp helper
timestamp() {
    date +"%Y-%m-%d %H:%M:%S"
}

# --- print helpers
print_info()  { echo -e "[INFO]  $1"; }
print_ok()    { echo -e "[OK]    $1"; }
print_warn()  { echo -e "[WARN]  $1"; }
print_fail()  { echo -e "[FAIL]  $1"; }

# --- log on first error
log_error() {
    [[ "$LOGGING" = false ]] && return
    if [[ $ERRORS -eq 0 ]]; then
        mkdir -p "$LOG_DIR"
        echo "[$(timestamp)]" >> "$LOG_FILE"
    fi
    echo -e "\t$1" >> "$LOG_FILE"
    ((ERRORS++))
}

can_sudo() {
    sudo -n true 2>/dev/null
}

##########################################################################
print_info "Checking required tools..."

for t in "${REQUIRED_TOOLS[@]}"; do
    if ! command -v "$t" >/dev/null 2>&1; then
        print_fail "Missing required tool: $t"
        echo "Please install $t before running again."
        exit 1   # Requirement: missing req tool → exit, don't log
    fi
done
print_ok "All required tools present"


##########################################################################
print_info "Checking network interface: $IFACE"

if ip link show "$IFACE" >/dev/null 2>&1; then
    print_ok "Interface detected"
else
    print_fail "Interface missing: $IFACE"
    log_error "Missing interface: $IFACE"
    # continue testing other layers
fi


##########################################################################
print_info "Checking IPv4 assignment..."

IPADDR=$(ip -4 addr show "$IFACE" 2>/dev/null | awk '/inet /{print $2}')
if [[ -n "$IPADDR" ]]; then
    print_ok "IPv4 address: $IPADDR"
else
    print_fail "No IPv4 address found"
    log_error "No IPv4 address on $IFACE"
fi


##########################################################################
print_info "Checking default gateway..."

GATEWAY=$(ip route | awk '/default/ {print $3}')
if [[ -n "$GATEWAY" ]]; then
    print_ok "Gateway: $GATEWAY"
    if ping -c1 -W2 "$GATEWAY" &>/dev/null; then
        print_ok "Gateway reachable"
    else
        print_fail "Gateway not reachable"
        log_error "Gateway unreachable: $GATEWAY"
    fi
else
    print_fail "No default gateway found"
    log_error "No default gateway"
fi


##########################################################################
print_info "Testing connectivity to public IP ($TEST_IP)..."

if ping -c1 -W2 "$TEST_IP" &>/dev/null; then
    print_ok "External IP reachable"
else
    print_fail "Unable to reach $TEST_IP"
    log_error "Cannot reach external IP: $TEST_IP"
fi


##########################################################################
print_info "Checking DNS configuration..."

NS=$(grep -E '^nameserver' /etc/resolv.conf 2>/dev/null)
if [[ -n "$NS" ]]; then
    print_ok "Nameservers found:"
    echo "$NS"
else
    print_fail "No nameservers found"
    log_error "No DNS nameservers"
fi


##########################################################################
print_info "Testing DNS resolution..."

if dig +short "$TEST_HOST" >/dev/null 2>&1; then
    print_ok "dig lookup succeeded ($TEST_HOST)"
else
    print_fail "dig DNS lookup failed ($TEST_HOST)"
    log_error "DNS resolution failed: $TEST_HOST"
fi


##########################################################################
print_info "Ping hostname..."

if ping -c1 -W2 "$TEST_HOST" &>/dev/null; then
    print_ok "Ping by hostname succeeded"
else
    print_fail "Ping by hostname failed"
    log_error "Failed to ping hostname: $TEST_HOST"
fi


##########################################################################
print_info "Testing HTTPS access..."

if curl -s --max-time 5 "https://$TEST_HOST" >/dev/null; then
    print_ok "HTTPS access OK"
else
    print_fail "HTTPS test failed"
    log_error "HTTPS connection failed: https://$TEST_HOST"
fi


##########################################################################
print_info "Optional privileged checks"

for t in "${OPTIONAL_TOOLS[@]}"; do
    if ! command -v "$t" >/dev/null 2>&1; then
        print_warn "$t not found — skipping"
        continue
    fi

    # requires sudo, do not log if no sudo
    if ! can_sudo; then
        print_warn "$t requires sudo — skipping"
        continue
    fi

    case "$t" in
        ufw)
            print_info "ufw status:"
            ufw status || true
        ;;
        iptables)
            print_info "iptables (top rules):"
            iptables -L -n | head -n 8 || true
        ;;
        traceroute)
            print_info "Traceroute to $TEST_HOST:"
            traceroute -w 2 "$TEST_HOST" || true
        ;;
    esac
done


##########################################################################
print_info "Network diagnostic COMPLETE"

if [[ $ERRORS -gt 0 ]]; then
    echo
    echo "❌ Found $ERRORS issue(s)"
    echo "Logged to: $LOG_FILE"
    exit 1
else
    echo
    echo "✅ No critical errors detected"
    exit 0
fi
